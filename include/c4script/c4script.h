/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2012-2016, The OpenClonk Team and contributors
 *
 * Distributed under the terms of the ISC license; see accompanying file
 * "COPYING" for details.
 *
 * "Clonk" is a registered trademark of Matthes Bender, used with permission.
 * See accompanying file "TRADEMARK" for details.
 *
 * To redistribute this file separately, substitute the full license texts
 * for the above references.
 */

#ifndef C4SCRIPTSTANDALONE_H
#define C4SCRIPTSTANDALONE_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

struct c4s_diagnostic_position
{
	const char *file, *function;
	uint64_t line, column, length;
	int valid;
};

typedef void (*c4s_errorhandlerfn)(void *ctx, const char *, struct c4s_diagnostic_position);

struct c4s_errorhandlers
{
	c4s_errorhandlerfn errors, warnings;
	void *ctx;
};

int c4s_runfile(const char *filename, struct c4s_errorhandlers *handlers);
int c4s_runstring(const char *script, struct c4s_errorhandlers *handlers);

int c4s_checkfile(const char *filename, struct c4s_errorhandlers *handlers);
int c4s_checkstring(const char *script, struct c4s_errorhandlers *handlers);

#ifdef __cplusplus
}
#endif

#endif // C4SCRIPTSTANDALONE_H
