/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Antony jr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @filename    : ZsyncWriter_p.hpp
 * @description : This is where the core of the delta writer is described.
*/
#ifndef ZSYNC_WRITER_PRIVATE_HPP_INCLUDED
#define ZSYNC_WRITER_PRIVATE_HPP_INCLUDED
#include <QBuffer>
#include <cmath>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDebug>
#include <QDir>
#include <QtEndian>
#include <QFileInfo>
#include <QtGlobal>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QScopedPointer>
#include <QTime>
#include <QTimer>
#include <QTemporaryFile>
#include <AppImageUpdaterBridgeErrorCodes.hpp>
#include <AppImageUpdaterBridgeStatusCodes.hpp>
#include <ZsyncInternalStructures_p.hpp>

namespace AppImageUpdaterBridge
{
class ZsyncWriterPrivate : public QObject
{
    Q_OBJECT
public:
    explicit ZsyncWriterPrivate(void);
    ~ZsyncWriterPrivate();

    static QString errorCodeToString(short);
    static QString statusCodeToString(short);
public Q_SLOTS:
    void setShowLog(bool);
    void setLoggerName(const QString&);
    qint32 getBytesWritten(void);
    void getBlockRanges(void);
    void rawSeqWrite(QByteArray*);
    void downloadFinished(void);
    void writeBlockRanges(qint32, qint32, QByteArray*);
    void setOutputDirectory(const QString&);
    void setConfiguration(qint32,qint32,qint32,qint32,qint32,qint32,
                          const QString&,const QString&,const QString&,QBuffer*,bool);
    void start(void);
    void cancel(void);

private Q_SLOTS:
    void doStart(void);
    void doCancel(void);
    void resetConnections(void);

#ifndef LOGGING_DISABLED
    void handleLogMessage(QString, QString);
#endif // LOGGING_DISABLED
    bool verifyAndConstructTargetFile(void);
    void addToRanges(zs_blockid);
    qint32 alreadyGotBlock(zs_blockid);
    qint32 buildHash(void);
    qint32 checkCheckSumsOnHashChain(const hash_entry *, const unsigned char *, qint32 );
    quint32 calcRHash(const hash_entry *const);
    void calcMd4Checksum(unsigned char *, const unsigned char*,size_t);
    zs_blockid getHashEntryBlockId(const hash_entry *);
    short tryOpenSourceFile(const QString&, QFile**);
    short parseTargetFileCheckSumBlocks(void);
    void writeBlocks(const unsigned char *, zs_blockid, zs_blockid);
    void removeBlockFromHash(zs_blockid);
    qint32 submitSourceData(unsigned char*, size_t, off_t);
    qint32 submitSourceFile(QFile*);
    qint32 rangeBeforeBlock(zs_blockid);
    zs_blockid nextKnownBlock(zs_blockid);

Q_SIGNALS:
    void initStart(void);
    void initCancel(void);
    void finishedConfiguring();
    void blockRange(qint32, qint32);
    void endOfBlockRanges(void);
    void blockRangesWritten(qint32, qint32, bool);
    void download(void);
    void started(void);
    void canceled(void);
    void finished(QJsonObject, QString);
    void progress(int percentage, qint64 bytesReceived, qint64 bytesTotal, double speed, QString units);
    void statusChanged(short);
    void error(short);
    void logger(QString, QString);
private:
    QAtomicInteger<bool> _bCancelRequested = false,
                         _bAcceptRange = true;
    QPair<rsum, rsum> _pCurrentWeakCheckSums = qMakePair(rsum({ 0, 0 }), rsum({ 0, 0 }));
    qint64 _nBytesWritten = 0;
    qint32 _nBlocks = 0,
           _nBlockSize = 0,
           _nBlockShift = 0, // log2(blocksize).
           _nContext = 0,    // precalculated blocksize * seq_matches.
           _nWeakCheckSumBytes = 0,
           _nStrongCheckSumBytes = 0, // # of bytes available for the strong checksum.
           _nSeqMatches = 0,
           _nSkip = 0,     // skip forward on next submit_source_data.
           _nTargetFileLength = 0;
    unsigned short _pWeakCheckSumMask = 0; // This will be applied to the first 16 bits of the weak checksum.

    const hash_entry *_pRover = nullptr,
                      *_pNextMatch = nullptr;
    zs_blockid _nNextKnown = 0;

    /* Hash table for rsync algorithm */
    quint32 _pHashMask = 0;
    hash_entry *_pBlockHashes = nullptr;
    hash_entry **_pRsumHash = nullptr;

    /* And a 1-bit per rsum value table to allow fast negative lookups for hash
     * values that don't occur in the target file. */
    quint32 _pBitHashMask = 0;
    unsigned char *_pBitHash = nullptr;

    qint32 _nRanges = 0;
    zs_blockid *_pRanges = nullptr; // Ranges needed to finish the under construction target file.
    QVector<QPair<qint32, qint32>> _pRequiredRanges;
    QScopedPointer<QBuffer> _pTargetFileCheckSumBlocks; // Checksum blocks that needs to be loaded into the memory.
    QScopedPointer<QCryptographicHash> _pMd4Ctx; // Md4 Hasher context.
    QString _sSourceFilePath,
            _sTargetFileName,
            _sTargetFileSHA1,
            _sOutputDirectory;
    QScopedPointer<QTemporaryFile> _pTargetFile; // Under construction target file.
    QScopedPointer<QTime> _pTransferSpeed;
#ifndef LOGGING_DISABLED
    QString _sLogBuffer,
            _sLoggerName;
    QScopedPointer<QDebug> _pLogger;
#endif // LOGGING_DISABLED 
};
}
#endif // ZSYNC_WRITER_PRIVATE_HPP_INCLUDED
