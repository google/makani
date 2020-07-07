C*********************************************************************** 
C    Module:  volts.f
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

C
C...VOLTS
C  Test of Xplot11 interface
C
C                          
      CHARACTER RDLINE*60
      DIMENSION X(26),Y(26)
      DATA X/0.,5.,15.,25.,30.,35.,40.,45.,50.,55.,60.,65.,70.,75.,80.,
     1      85.,90.,95.,100.,105.,110.,115.,120.,125.,2*0./
C
      DATA Y/0.,10.,15.,10.,-10.,-50.,-80.,-110.,-130.,-145.,-155.,
     1      -160.,-158.,-150.,-130.,-90.,-70.,-20.,20.,50.,70.,80.,85.,
     2      90.,2*0./
C
C
    5 FORMAT(A)
C
C...Alter actual data scales
      DO 10 I=1,24
      X(I) = X(I) * 1000
 10   Y(I) = Y(I) / 10000
C
      IPD = 1
      RELSIZ = -0.6
C
C...Select type(s) of plot output
      WRITE(*,40)
 40   FORMAT('Input plot device (1 screen, 2 PShardcopy, 3 both): ',$)
      READ(*,5,end=1000) RDLINE
      IF(RDLINE.NE.' ') READ(RDLINE,*,err=42) IPD
C
C...Get plot window size as fraction of root window
 42   WRITE(*,45)
 45   FORMAT('Enter window relative size (<ret> gives -0.6): ',$)
      READ(*,5,end=1000) RDLINE
      IF(RDLINE.NE.' ') READ(RDLINE,*,err=50) RELSIZ
C
 50   CALL PLINITIALIZE
      ipslu = 0
      CALL PLOPEN(RELSIZ,ipslu,IPD)
C
      CALL NEWFACTOR(0.9)
      CALL PLOT(1.05,1.05,-3)
C
C...Scale factors from array coordinates
      CALL SCALE(X,7.,24,+1)
      CALL SCALE(Y,7.,24,+1)
C
      IMASK = -1
      IMASK = -30584
C...Test for line mask
c      WRITE(*,*) 'Enter line pattern bit mask (integer)'
c      READ(*,*,end=1000) IMASK
      call NEWPAT(IMASK)
C
C...Plot the array of points
      CALL NEWPEN (3)
      CALL NEWCOLORNAME('red')
      CALL LINE (X,Y,24,1,+1,0)
      MSKALL = -1
      call NEWPAT(MSKALL)
C
C...Label the axes
      CALL NEWPEN (2)
      CALL NEWCOLORNAME('orchid')
      CALL AXIS (0.,0.,'NANOSECONDS',-11,7.,0.,X(25),X(26))
      CALL NEWCOLORRGB(0,255,255)
      CALL AXIS (0.,0.,'MILLIVOLTS',+10,7.,90.,Y(25),Y(26))
C
C...Plot legend
      CALL NEWPEN (1)
      CALL NEWCOLORNAME('cadetblue')
      CALL PLCHAR (.5,.5,.1,'DX = ',0.,+5)
      CALL PLNUMB (999.,999.,.1,X(26),0.,+3)
      CALL PLNUMB (1.,.75,.1,X(26),0.,0)
      CALL PLNUMB (1.,1.,.1,X(26),0.,-1)
      CALL PLNUMB (1.,1.25,.1,X(26),0.,-4)
C
C...Titles
      CALL NEWCOLORNAME('lime green')
      CALL PLCHAR (2.3,6.5,.1,'VERSAPLOT SAMPLE',0.,+16)
      CALL PLCHAR (1.5,6.75,.2,'TIME VS VOLTAGE',0.,+15)
      CALL NEWCOLORNAME('black')
C
      CALL PLEND
      write(*,*) 'Hit return to exit'
      READ(*,999)
C
 999  FORMAT(A)     
1000  CALL PLCLOSE
      STOP
      END


