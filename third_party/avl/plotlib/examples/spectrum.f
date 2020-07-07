C*********************************************************************** 
C    Module:  spectrum.f
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


      program spectrum
C
C   Displays chosen COLORSPECTRUMDEFAULT colors in a circle and bar
C
C
      dimension xp(100), yp(100)
C
      CHARACTER*12 HUES
      CHARACTER*4 INP
      CH = 0.02
C
      PI = 4.0*ATAN(1.0)
C
C---Decide about what devices to plot to
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
      CALL PLINITIALIZE
C
C---Now, how many colors...
      WRITE(*,*) ' Enter number of colors'
      READ(*,1000,end=2000) INP
      ncolors = 128
      if(INP.ne.' ') then
        READ(INP,*,end=2000,err=2000) ncolors
      endif
C
      WRITE(*,*) ' Specify hue string (out of ROYGCBM)'
      READ (*,1000) HUES
      IF(HUES.EQ.' ') HUES = 'ROYGCBM'
C
C---Set up colormap spectrum colors
      if(ncolors.LE.1) ncolors = 0
      CALL COLORSPECTRUMHUES(ncolors,HUES)
C
C---- radius of circle
      rad = 3.0
c
      CALL PLOPEN(0.7,ipslu,IDEV)
      CALL PLOT(1.3*rad,1.3*rad,-3)
c
c---- plot circle
      do ii = 1,ncolors
        call NEWCOLOR(-ii)
        t0 = float(ii-1)/float(ncolors) ! + 0.167
        t1 = float(ii  )/float(ncolors) ! + 0.167
C
        xp(1) = 0.0
        yp(1) = 0.0
        xp(2) = rad*cos(2.0*pi*t0)
        yp(2) = rad*sin(2.0*pi*t0)
        xp(3) = rad*cos(2.0*pi*(t0+0.25*(t1-t0)))
        yp(3) = rad*sin(2.0*pi*(t0+0.25*(t1-t0)))
        xp(4) = rad*cos(2.0*pi*(t0+0.50*(t1-t0)))
        yp(4) = rad*sin(2.0*pi*(t0+0.50*(t1-t0)))
        xp(5) = rad*cos(2.0*pi*(t0+0.75*(t1-t0)))
        yp(5) = rad*sin(2.0*pi*(t0+0.75*(t1-t0)))
        xp(6) = rad*cos(2.0*pi*t1)
        yp(6) = rad*sin(2.0*pi*t1)
        call POLYLINE(xp,yp,6,1)
      end do
C
      CALL PLOT(1.5*rad,-rad,-3)
C
c---- plot bar
      dx = 1.0
      dy = 2.0*rad/float(ncolors)
      do ii = 1,ncolors
        call NEWCOLOR(-ii)
c
        x0 = 0.0
        y0 = dy*float(ii-1)
c
        xp(1) = x0
        yp(1) = y0
        xp(2) = x0+dx
        yp(2) = y0
        xp(3) = x0+dx
        yp(3) = y0+dy
        xp(4) = x0
        yp(4) = y0+dy
        call POLYLINE(xp,yp,4,1)
      end do
C
      CALL NEWCOLORNAME('black')
      CALL PLOTABS(1.,.75,-3)
      CALL PLCHAR (999.,999.,.1,'SPECTRUM  ',0.,+10)
      CALL PLCHAR (999.,999.,.1,HUES,0.,LEN(HUES))
      CALL PLOTABS(1.,0.5,-3)
      CALL PLCHAR (999.,999.,.1,'Ncolors = ',0.,+10)
      CALL PLNUMB (999.,999.,.1,FLOAT(ncolors),0.,-1)
C
      CALL PLFLUSH
      WRITE(*,*) 'Hit return to test replot'
      READ(5,1000) DUMMY
C
      CALL REPLOT(IDEVRP)
      CALL PLFLUSH
C
      WRITE(*,*) 'Hit return to end test'
      READ(5,1000) DUMMY
 1000 FORMAT(A)
C
C      GO TO 1
C
 2000 CALL PLOT(0.0,0.0,+999)
      STOP
      END
