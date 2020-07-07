C*********************************************************************** 
C    Module:  gridtest.f
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

C Example/Test of grid routine
C  Sets up two plots, each containing a grid with a label and symbol line
C  First  plot comes up as B&W in portrait window
C  Second plot comes up as color in larger landscape window
C
      INCLUDE 'colors.inc'
      CHARACTER INP*10
      DATA LMASK1, LMASK2 / -32640, -30584 /
      DATA LMASK3, LMASK4/  -21846, Z'AAAAAAAA'/
C
C---- number of grid intervals per axis annotation interval
      NGR = 2
C
      XMIN=.1
      YMIN=.1
      XMAX=.9
      YMAX=.6
      XDEL = 0.1
      YDEL = 0.1
      XSF = 1.0
      YSF = 1.0
      NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
      NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
      DXG = XSF*XDEL / FLOAT(NGR)
      DYG = YSF*YDEL / FLOAT(NGR)
C
      CH = 0.03
      SH = 0.2*CH
C
C---What devices to plot to?
      WRITE(*,*) ' '
 1    WRITE(*,*) ' Enter -1 for no PS, 0 for B&W PS, 1 for color PS'
      READ(*,1000,end=2000) INP
      IOUT = -1
      if(INP.NE.' ') READ(INP,*,end=2000,err=2000) IOUT
 2    IF(IOUT.LT.0) IDEV = 1
      IF(IOUT.EQ.0) IDEV = 3
      IF(IOUT.GE.1) IDEV = 5
C
C---plot #1 in B&W portrait mode 0.5 of screen height
      CALL PLINITIALIZE
      CALL PLOPEN(-0.5,0,IDEV)
      CALL PLOT(0.1,0.1,-3)
      CALL NEWPEN(1)
      CALL NEWFACTOR(10.)
      CALL PLCHAR(0.2,0.2,1.2*CH,'TEST FOR GRID',0.0,13)
      CALL PLSYMB(999.,0.2,CH,1,0.0,0)
      CALL PLSYMB(0.2,0.2,CH,1,0.0,-1)
      CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK4)
      CALL PLFLUSH
      PAUSE
      CALL PLOT(0.,0.,-999)
C
C---plot #2 in landscape mode 0.7 of screen width
C   (green grid with red lettering with blue symbol line)
      CALL PLOPEN(0.7,0,IDEV)
      CALL PLOT(0.1,0.1,-3)
      CALL NEWFACTOR(10.)
      CALL NEWPEN(1)
      NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
      NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
      DXG = XSF*XDEL / FLOAT(NGR)
      DYG = YSF*YDEL / FLOAT(NGR)

      call NEWCOLORNAME('green')
      CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2)

      CALL NEWPEN(1)
      CALL NEWCOLORRGB(255,0,0)
      CALL PLCHAR(0.2,0.2,1.2*CH,'TEST FOR GRID',0.0,13)
      call NEWCOLOR(BLUE)
      call NEWPEN(1)
      CALL PLSYMB(999.,0.2,CH,1,0.0,0)
      CALL PLSYMB(0.2,0.2,CH,1,0.0,-1)
      CALL PLFLUSH
      PAUSE
      CALL PLCLOSE
C
 1000 FORMAT(A)
 2000 STOP
      END




