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
#include "test.h"

int g_checks = 0;
int g_failures = 0;

struct Suite {
  const char *name;
  void (*fn)();
};

int main() {
  const Suite suites[] = {
      {"gears", TestGears},         {"interlock", TestInterlock},
      {"lookup", TestLookup},       {"eds", TestEds},
      {"max22200", TestMax22200},   {"sas", TestSas},
      {"engage", TestEngage},       {"speedsens", TestSpeedSens},
      {"tcu", TestTcu},             {"valveout", TestValveOut},
      {"anaconv", TestAnaConv},     {"linpump", TestLinPump},
      {"shifter", TestShifter},     {"canif", TestCanIf},
      {"params", TestParams},       {"lever", TestLever},
  };
  for (const Suite &s : suites) {
    int before = g_failures;
    s.fn();
    printf("%-10s %s\n", s.name, g_failures == before ? "ok" : "FAILED");
  }
  printf("%d checks, %d failed\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
