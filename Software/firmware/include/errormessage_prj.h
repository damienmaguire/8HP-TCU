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
#ifndef ERRORMESSAGE_PRJ_H_INCLUDED
#define ERRORMESSAGE_PRJ_H_INCLUDED

#define ERROR_BUF_SIZE 16

#define ERROR_MESSAGE_LIST                                                     \
  ERROR_MESSAGE_ENTRY(CANTIMEOUT, ERROR_DISPLAY)                               \
  ERROR_MESSAGE_ENTRY(DRVFAULT, ERROR_STOP)                                    \
  ERROR_MESSAGE_ENTRY(DRVLOST, ERROR_STOP)                                     \
  ERROR_MESSAGE_ENTRY(DRVINIT, ERROR_STOP)                                     \
  ERROR_MESSAGE_ENTRY(VALVEMAP, ERROR_STOP)                                    \
  ERROR_MESSAGE_ENTRY(AUXVALVE, ERROR_DISPLAY)                                 \
  ERROR_MESSAGE_ENTRY(INTERLOCK, ERROR_STOP)                                   \
  ERROR_MESSAGE_ENTRY(UNDERVOLT, ERROR_STOP)                                   \
  ERROR_MESSAGE_ENTRY(SPEEDSENS, ERROR_DISPLAY)                                \
  ERROR_MESSAGE_ENTRY(GEARRATIO, ERROR_STOP)                                   \
  ERROR_MESSAGE_ENTRY(ATFSENSOR, ERROR_DISPLAY)

#endif // ERRORMESSAGE_PRJ_H_INCLUDED
