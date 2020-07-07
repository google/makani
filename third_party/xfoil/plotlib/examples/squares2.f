C*********************************************************************** 
C    Module:  squares.f
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

C---Test of filled polyline plotting
C   Displays a sine wave of filled color rectangles
      dimension xp(100), yp(100), x(100), y(100)
C
      CHARACTER*4 INP
      CH = 0.02
C
      x(1) = 0.
      y(1) = 0.
      x(2) = 0.5
      y(2) = 0.
      x(3) = 0.5
      y(3) = 0.3
      x(4) = 0.
      y(4) = 0.3
      x(5) = x(1)
      y(5) = y(1)
      n = 5
C
C---Decide about what devices to plot to
      WRITE(*,*) ' '
      WRITE(*,*) 'SQUARES Plot test'
      WRITE(*,*) ' You may just <cr> for each question to take defaults'
      WRITE(*,*) ' '
   1  WRITE(*,*) ' Enter -1 for no PS, 0 for B&W PS, 1 for color PS'
      READ(*,1000,end=2000) INP
      ips = -1
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ips
      endif
      IDEV = 1
      IF(ips.eq.0) IDEV = 3
      IF(ips.ge.1) IDEV = 5
      ipslu = 0
C
C---- for REPLOT:  X11 only
      IDEVRP = 1
C
C
      CALL PLINITIALIZE
C
C---Now, how many colors...
      WRITE(*,*) ' Enter # colors (0 or 1 gives no colors)'
      READ(*,1000,end=2000) INP
      ncolors = 64
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ncolors
      endif
C---Set up colormap spectrum colors
      if(ncolors.LE.1) ncolors = 0
      CALL COLORSPECTRUMHUES(ncolors,'MBCGYR')
C
      DO III = -1,1,2
C
      CALL PLOPEN(float(III)*0.7,ipslu,IDEV)
C
      CALL PLOT(0.5,0.5,-3)
      do ii = 1,ncolors
        call NEWCOLOR(-ii)
        f = float(ii-1)/float(ncolors-1)
        xx = 9.0*f
        yy = 3.0*sin(2.0*3.1416*f)
        do i = 1, n
          xp(i) = x(i) + xx
          yp(i) = 4.0 + y(i) + yy
        end do
        call POLYLINE(xp,yp,n,1)
      end do
C
      CALL NEWCOLORNAME('green')
      CALL PLCHAR(0.,0.,10.*CH,'Test ',0.0,5) 
      CALL NEWCOLORRGB(0,0,255)
      CALL PLCHAR(999.,999.,10.*CH,'of ',0.0,3) 
      CALL NEWCOLORNAME('yellow')
      CALL PLCHAR(999.,999.,10.*CH,'Color ',0.0,6)
      CALL NEWCOLORNAME('red')
      CALL PLCHAR(999.,999.,10.*CH,'Fill',0.0,4)
C
      CALL PLFLUSH
C
      WRITE(*,*) 'Hit return to test replot'
      READ(5,1000) DUMMY
      END DO
C
      WRITE(*,*) 'Hit return to end test'
      READ(5,1000) DUMMY
 1000 FORMAT(A)
C
 2000 CALL PLCLOSE
      STOP
      END












