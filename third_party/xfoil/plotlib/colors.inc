C***********************************************************************
C    Module:  colors.inc
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
C     Version 4.41  7/5/98
C
C******************************************
C  Default color definitions for Xplot11
C  For use as arguments to NEWCOLOR, e.g.
C
C      CALL NEWCOLOR(GREEN)
C
C
C  Alternatively, can also use...
C
C      CALL NEWCOLORNAME('GREEN')
C
C******************************************
C
      Integer BLACK,  
     &        WHITE,  
     &        RED,    
     &        ORANGE, 
     &        YELLOW, 
     &        GREEN,  
     &        CYAN,   
     &        BLUE,   
     &        VIOLET,
     &        MAGENTA
C
      parameter (BLACK   =  1)
      parameter (WHITE   =  2)
      parameter (RED     =  3)
      parameter (ORANGE  =  4)
      parameter (YELLOW  =  5)
      parameter (GREEN   =  6)
      parameter (CYAN    =  7)
      parameter (BLUE    =  8)
      parameter (VIOLET  =  9)
      parameter (MAGENTA =  10)
