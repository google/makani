C***********************************************************************
C    Module:  masks.inc
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
C     Version 4.46 11/28/01
C
C---- dot-pattern masks for use with PLGRID, NEWPAT, etc.
C
C     mask0:  _________________________   (solid)
C         1:  .........................
C         2:  . . . . . . . . . . . . .
C         3:  .   .   .   .   .   .   .
C         4:  .       .       .       .
C
      data  mask0,  mask1,  mask2,  mask3,  mask4
     &   / -1    , -21846, -30584, -32640, -32768 /
C
C      data  mask0,      mask1,      mask2,      mask3,      mask4
C     &   /Z'FFFFFFFF',Z'AAAAAAAA',Z'CCCCCCCC',Z'88888888',Z'21084210'/



