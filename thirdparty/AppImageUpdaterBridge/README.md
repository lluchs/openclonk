<p align="center">
  <img src=".img/poster.png" height="400px" width=auto alt="AppImageUpdaterBridge Poster">  <br>
</p>


# AppImage Updater Bridge [![GitHub issues](https://img.shields.io/github/issues/antony-jr/AppImageUpdaterBridge.svg?style=flat-square)](https://github.com/antony-jr/AppImageUpdaterBridge/issues) [![GitHub stars](https://img.shields.io/github/stars/antony-jr/AppImageUpdaterBridge.svg?style=flat-square)](https://github.com/antony-jr/AppImageUpdaterBridge/stargazers) [![GitHub license](https://img.shields.io/github/license/antony-jr/AppImageUpdaterBridge.svg?style=flat-square)](https://github.com/antony-jr/AppImageUpdaterBridge/blob/master/LICENSE) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/8ec8eac35a304883829b785d298b6fa6)](https://www.codacy.com/app/antony-jr/AppImageUpdaterBridge?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=antony-jr/AppImageUpdaterBridge&amp;utm_campaign=Badge_Grade) [![Travis Build Status](https://api.travis-ci.org/antony-jr/AppImageUpdaterBridge.svg?branch=master)](https://travis-ci.org/antony-jr/AppImageUpdaterBridge)


AppImage Updater Bridge is a **delta updater** based on the *zsync algorithm* for the *AppImage format* , Using this library you can 
delta update any AppImage file , That is , Only download the binary data which you need to get the latest version , 
Therefore saving your time and internet.

AppImage Updater Bridge is not the **official library** to do this stuff , The official library is [here](https://github.com/AppImage/AppImageUpdate) which is also written in **C++** but with no real **Qt support** , Thats why this library is built.

This library gives absolute support for *Qt* , Infact , All you need is the minimal installation of *Qt*(i.e, base ).


## Features

* *Single Threaded , Non-Blocking API* - Using Qt's Event Loop.(Optionally , You can use a seperate thread.)

* *Easy to Use API* - Made some effort to match Qt's style.

* *Cyclic API , No use of mutex* - The whole library is fully cyclic and therefore no mutex is used.(Only signals and slots.)

* *Pure C++* - Ported most of the legacy Zsync code to C++.

* *No third party libraries needed* - The reason why you want to use this library.

* *Drink the Qt Kool-aid* - In a positive way.


**Witness it with your own eyes,**

```
#include <QCoreApplication>
#include <AppImageUpdaterBridge>

using namespace AppImageUpdaterBridge;

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    AppImageDeltaRevisioner DeltaRevisioner;
    QObject::connect(&DeltaRevisioner, &AppImageDeltaRevisioner::finished, &app, &QCoreApplication::quit);
    DeltaRevisioner.setShowLog(true);
    DeltaRevisioner.start();
    return app.exec();
}
```


# Try it

See this library in **action** at this [repo](https://github.com/antony-jr/AppImageUpdater) which reimplements the official **AppImageUpdater Tool** using this library.



# Getting Started

Please refer the official [documentation](https://antony-jr.github.io/AppImageUpdaterBridge).

# Contributors [![AIUB Contributors](https://img.shields.io/github/contributors/antony-jr/AppImageUpdaterBridge.svg)](https://github.com/antony-jr/AppImageUpdaterBridge/graphs/contributors)

These are the people who helped to make the bridge stronger! :heart:

<table>
    <tr align="center">
        <td>
            <img src="https://avatars2.githubusercontent.com/u/1092613?v=4" width="100px"><br>
            <sub>
                <strong>
                    <a href="https://github.com/technic">technic93</a>
                </strong>
            </sub><br>
            <a href="https://github.com/antony-jr/AppImageUpdaterBridge/commits?author=technic">💻</a>
      </td>
    </tr>
</table>


# Acknowledgements ![Thank You](https://img.shields.io/badge/Always-Say%20Thank%20You!-blue.svg?style=flat-square)

* [AppImages](https://github.com/AppImage) - Motivation to start this project.
* [zsync](https://github.com/cph6/zsync) ([Colin Phipps](https://github.com/cph6)) - Zsync algorithm's author.
* [zsync2](https://github.com/AppImage/zsync2) ([@TheAssassin](https://github.com/TheAssassin)) - Helpful references.
* [Qt](https://github.com/qt) - And Everything else.


# Support [![Twitter](https://img.shields.io/twitter/url/https/github.com/antony-jr/AppImageUpdaterBridge.svg?style=social)](https://twitter.com/intent/tweet?text=Checkout%20%23AppImage%20Updater%20Bridge%20at%20https%3A%2F%2Fgithub.com%2Fantony-jr%2FAppImageUpdaterBridge)

If you think that this project is **cool** then you can give it a :star: or :fork_and_knife: it if you want to improve it with me. I really :heart: stars though!   

# License

The BSD 3-clause "New" or "Revised" License.

Copyright (C) 2017 , antony jr.   
All Rights Reserved.
