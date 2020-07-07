C*********************************************************************** 
C    Module:  symbols.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This program is free software; you can redistribute it and/or modify 
C    it under the terms of the GNU General Public License as published by 
C    the Free Software Foundation; either version 2 of the License, or 
C    (at your option) any later version. 
C 
C    This program is distributed in the hope that it will be useful, 
C    but WITHOUT ANY WARRANTY; without even the implied warranty of 
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
C    GNU General Public License for more details. 
C 
C    You should have received a copy of the GNU General Public License 
C    along with this program; if not, write to the Free Software 
C    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
C
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C*********************************************************************** 

C---Test routine for Pltlib
C   Displays a symbol set in color
C
      CHARACTER*4 INP
      CH = 0.02
C
C---Decide about what devices to plot to
      WRITE(*,*) ' '
      WRITE(*,*) 'SYMBOLS Plot test'
      WRITE(*,*) ' You may just <cr> for each question to take defaults'
      WRITE(*,*) ' '
   1  WRITE(*,*) ' Enter -1 for no PS, 0 for B&W PS, 1 for color PS'
      READ(*,1000,end=2000) INP
      ihard = -1
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ihard
      endif
      IDEV = 1
      IF(ihard.eq.0) IDEV = 3
      IF(ihard.ge.1) IDEV = 5
      ipslu = 0
C
C---Initialize the plot package before we get into color plotting...
      CALL PLINITIALIZE
C
C---Now, how many colors...
      WRITE(*,*) ' Enter # colors (0 or 1 gives no colors)'
      READ(*,1000,end=2000) INP
      ncolors = 32
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ncolors
      endif
C---Set up colormap spectrum colors
      if(ncolors.LE.1) ncolors = 1
      CALL COLORSPECTRUMHUES(ncolors,'MBCGYR')
C 
C---Take the default window (portrait, 2/3 screen dimension)
      CALL PLOPEN(0.,ipslu,IDEV)
C
      CALL NEWFACTOR(5.0)
      CALL PLOT(0.10,0.1,-3)
ccc      CALL NEWCOLORNAME('black')
C
C---Plot the symbols in 8 columns of 32 characters each (256 total)
      DO ISET=1, 8
C
        I0 = (ISET-1)*32 + 1
        IN = I0 + 31
C
        DO I=I0, IN
          RNUM = FLOAT(I-1)
          XX = 0.2*FLOAT(ISET-1)
          YY = (36.-FLOAT(I-I0))*2.0*CH
C
          ICOLOR = MOD(I-1,NCOLORS) + 1
C         WRITE(*,*) 'ICOLOR,ISYM ',ICOLOR,I-1
C---Select one of the colormap spectrum colors (repeat, modulo ncolors)
          IF(ncolors.GT.1) CALL NEWCOLOR(-icolor)
          CALL PLNUMB(XX,YY-0.5*CH,CH,RNUM,0.0,-1)
          CALL PLCHAR(XX+4.0*CH,YY,CH,char(I-1),0.0,1)
        END DO
      END DO
C
C---Put colored title at bottom of plot
      CALL NEWCOLORNAME('blue')
      CALL PLCHAR(0.,0.,2.*CH,'Xplot11 ',0.0,8)
      CALL NEWCOLORNAME('green')
      CALL PLCHAR(999.,999.,2.*CH,'PLCHAR ',0.0,7)
      CALL NEWCOLORNAME('red')
      CALL PLCHAR(999.,999.,2.*CH,'test',0.0,4)
C
      CALL PLFLUSH
      WRITE(*,*) 'Hit return to end test'
      READ(5,1000) INP
 1000 FORMAT(A)
C
      CALL PLEND
C      GO TO 1
C
 2000 CALL PLCLOSE
      STOP
      END



















