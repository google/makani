C*********************************************************************** 
C    Module:  volts_old.f
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

C VOLTS
C
C Old-style Versatec calls test routine
C
      LOGICAL LODD
      DIMENSION X(26),Y(26),RED(9),BLUE(9),GREEN(9),NCOL(2)
      DATA X/0.,5.,15.,25.,30.,35.,40.,45.,50.,55.,60.,65.,70.,75.,80.,
     1      85.,90.,95.,100.,105.,110.,115.,120.,125.,2*0./
C
      DATA Y/0.,10.,15.,10.,-10.,-50.,-80.,-110.,-130.,-145.,-155.,
     1      -160.,-158.,-150.,-130.,-90.,-70.,-20.,20.,50.,70.,80.,85.,
     2      90.,2*0./
      DATA NCOL/0,8/
      DATA RED  /0.0,0.0,0.0,1.0,0.0,1.0,1.0,0.0,1.0/
      DATA BLUE /0.0,1.0,0.0,0.0,1.0,1.0,0.0,0.0,1.0/
      DATA GREEN/0.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0/	
C
C
C...ALTER ACTUAL DATA SCALES
      DO 10 I=1,24
      X(I) = X(I) * 1000
   10 Y(I) = Y(I) / 10000
C
      IDUM = 0
      ICNT = 0
      IPD  = 6
 100  CALL PLOTS (IDUM,0,IPD)
C
      LODD = (MOD(ICNT,2).EQ.1)
      CALL FACTOR (.9)
      IF(IPD .EQ. 6) THEN
       DO 20 I=1,NCOL(2)
         IF(.NOT.LODD) CALL SETCOL(I-1,RED(I),GREEN(I),BLUE(I))
         IF(LODD)      CALL SETCOL(I-1,RED(9-I),GREEN(9-I),BLUE(9-I))
 20    CONTINUE
      ENDIF
C
      CALL PLOT (1.05,1.05,-3)
C
      CALL SCALE (X,7.,24,+1)
      CALL SCALE (Y,7.,24,+1)
C
      CALL COLOR(2)
      CALL NEWPEN (2)
cc      CALL LINE (X,Y,24,1,1,0)
      CALL LINE (X,Y,24,1,1,0)
      CALL NEWPEN (1)
      CALL COLOR(0)
C
      CALL AXIS (0.,0.,'NANOSECONDS',-11,7.,0.,X(25),X(26))
      CALL AXIS (0.,0.,'MILLIVOLTS',+10,7.,90.,Y(25),Y(26))
C
      CALL SYMBOL (.5,.5,.1,'DX = ',0.,+5)
      CALL NUMBER (999.,999.,.1,X(26),0.,+3)
      CALL NUMBER (1.,.75,.1,X(26),0.,0)
      CALL NUMBER (1.,1.,.1,X(26),0.,-1)
      CALL NUMBER (1.,1.25,.1,X(26),0.,-4)
C
      CALL COLOR(4)
      CALL SYMBOL (2.3,6.5,.1,'VERSAPLOT SAMPLE',0.,+16)
      CALL SYMBOL (1.5,6.75,.2,'TIME VS VOLTAGE',0.,+15)
      CALL COLOR(0)
      CALL PLOTOF
C
      ICNT = ICNT+1
      CALL PLOT (0.,0.,-999)
      WRITE(*,*) '<cr> to cycle colors'
      READ(*,*)
      if(ICNT.LE.2) go to 100
      
      CALL PLOT (0.,0.,999)
      STOP
      END
















