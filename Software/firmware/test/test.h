/*
 * This file is part of the 8HP-TCU project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef TEST_H_INCLUDED
#define TEST_H_INCLUDED

/* Minimal test helpers: a failed check prints where and what, and the
 * run returns non-zero if anything failed. */
#include <stdio.h>

extern int g_checks;
extern int g_failures;

#define CHECK(c)                                                               \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(c)) {                                                                \
      g_failures++;                                                            \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                      \
    }                                                                          \
  } while (0)

#define CHECK_EQ(a, b)                                                         \
  do {                                                                         \
    g_checks++;                                                                \
    long long _a = (long long)(a), _b = (long long)(b);                        \
    if (_a != _b) {                                                            \
      g_failures++;                                                            \
      printf("FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__, __LINE__, #a,  \
             #b, _a, _b);                                                      \
    }                                                                          \
  } while (0)

void TestGears();
void TestInterlock();
void TestLookup();
void TestEds();
void TestMax22200();
void TestSas();
void TestEngage();
void TestSpeedSens();
void TestTcu();
void TestValveOut();

#endif // TEST_H_INCLUDED
