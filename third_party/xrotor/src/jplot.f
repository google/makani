C***********************************************************************
C    Module:  jplot.f
C 
C    Copyright (C) 2011 Mark Drela 
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
C***********************************************************************
      PROGRAM JPLOT
C------------------------------------------------------
C     Plots an operating map of Cp vs J with contours 
C     of efficiency and blade angle from subroutine JMAP
C
C     Usage:  % jplot filename
C
C------------------------------------------------------
C
      PARAMETER (LX=91,KX=81)
      REAL EF(LX,KX), BE(LX,KX)
      REAL RJ(LX,KX), CP(LX,KX)
      REAL RJ0(LX), CP0(KX)
      INTEGER KLOW(LX), KUPP(LX)
      LOGICAL LABCON, YES
      CHARACTER*80 FNAME
      CHARACTER*32 NAME
      CHARACTER*4 DUMMY
C
C
 1    FORMAT(A)
C
C---- plotting parameters
      CS  = 0.015        ! character height/SIZE
      PAR = 0.7          ! y-axis height/SIZE
C
C---- Plotting flag
      IDEV = 1   ! X11 window only
c     IDEV = 2   ! B&W PostScript output file only (no color)
c     IDEV = 3   ! both X11 and B&W PostScript file
c     IDEV = 4   ! Color PostScript output file only 
c     IDEV = 5   ! both X11 and Color PostScript file 
C
C---- Re-plotting flag (for hardcopy)
      IDEVRPB = 2   ! B&W PostScript
      IDEVRPC = 4   ! Color PostScript
C
C---- PostScript output logical unit and file specification
      IPSLU = 0  ! output to file  plot.ps   on LU 4    (default case)
c     IPSLU = ?  ! output to file  plot?.ps  on LU 10+?
C
C---- screen fraction taken up by plot window upon opening
      SCRNFR = 0.60
C
C---- Default plot size in inches
C-    (Default plot window is 11.0 x 8.5)
      SIZE = 7.5
C
      CALL PLINITIALIZE
C
C---- set up color spectrum
      NCOLOR = 64
      CALL COLORSPECTRUMHUES(NCOLOR,'RYGCBM')
C
C
C---- axis tick mark increments for J, Cp
      DRJ = 0.5
      DCP = 0.05
C
C---- grid increments for J, Cp
      GRJ = 0.2
      GCP = 0.02
C
C--- file name from command line or ask user
      CALL GETARG(1,FNAME)
      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter dump filename^',FNAME)
C
C--- alternate for Windoze DVFortran
ccc   CALL GETFNAME(FNAME)
C
      OPEN(11,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED')
C
      READ(11) NAME
      READ(11) RE6, AMACH
      READ(11) NRJ, NCP
      READ(11) (KLOW(L),L=1,NRJ)
      READ(11) (KUPP(L),L=1,NRJ)
      READ(11) (RJ0(L),L=1,NRJ)
      READ(11) (CP0(K),K=1,NCP)
C
      WRITE(*,1)  ' '
      WRITE(*,1) NAME
      WRITE(*,*) '  RE = ',RE6*1.0E6
      WRITE(*,*) 'MACH = ',AMACH
      WRITE(*,*) '#J  = ',NRJ
      WRITE(*,*) '#Cp = ',NCP
C
      PAUSE 'Hit return to see J values'
      DO I = 1, NRJ
        WRITE(*,*) I,RJ0(I)
      END DO
C
      PAUSE 'Hit return to see CP values'
      DO I = 1, NCP
        WRITE(*,*) I,CP0(I)
      END DO
C
C
      DO 21 K=1, NCP
        READ(11) (EF(L,K),L=1,NRJ)
   21 CONTINUE
C
      DO 22 K=1, NCP
        READ(11) (BE(L,K),L=1,NRJ)
   22 CONTINUE
C
      CLOSE(11)
C
      CALL ASKL('Dump data to formatted tables?^',YES)
      IF(YES) THEN
C--- Dump efficiency map
        FNAME = 'eff.map'
cc        CALL ASKS('Enter efficency output filename^',FNAME)
        OPEN(11,FILE=FNAME,STATUS='UNKNOWN',FORM='FORMATTED')
        WRITE(11,*) NRJ,NCP
        WRITE(11,10) FLOAT(NCP),(CP0(K),K=1,NCP)
        DO L = 1, NRJ
          WRITE(11,10) RJ0(L),(EF(L,K),K=1,NCP)
        END DO
        CLOSE(11)
C
C--- Dump prop pitch map
        FNAME = 'pitch.map'
cc        CALL ASKS('Enter efficency output filename^',FNAME)
        OPEN(11,FILE=FNAME,STATUS='UNKNOWN',FORM='FORMATTED')
        WRITE(11,*) NRJ,NCP
        WRITE(11,10) FLOAT(NCP),(CP0(K),K=1,NCP)
        DO L = 1, NRJ
          WRITE(11,10) RJ0(L),(EF(L,K),K=1,NCP)
        END DO
        CLOSE(11)
C
 10     FORMAT(30(G12.5,','))
C
C--- Dump file for GNUPLOT
        FNAME = 'eff.gnuplot'
        OPEN(11,FILE=FNAME,STATUS='UNKNOWN',FORM='FORMATTED')
        DO L = 1, NRJ
         DO K = 1, NCP
          WRITE(11,*) RJ0(L),CP0(K),EF(L,K)
         END DO
          WRITE(11,*) ' '
        END DO
        CLOSE(11)
C
      ENDIF
C
      CALL ASKL('Proceed to plot JMAP?^',YES)
      IF(YES) THEN
C
C---- set plot axis limits
      WRITE(*,*) 'J  limits:', RJ0(1), RJ0(NRJ)
      WRITE(*,*) 'Cp limits:', CP0(1), CP0(NCP)
      CALL ASKR('Enter max J  annotation (0 for auto-scale)^',RJMAX)
      CALL ASKR('Enter max Cp annotation (0 for auto-scale)^',CPMAX)
C
      IF(RJMAX.EQ.0.0) RJMAX = DRJ * FLOAT( INT(RJ0(NRJ)/DRJ + 1.1) )
      IF(CPMAX.EQ.0.0) CPMAX = DCP * FLOAT( INT(CP0(NCP)/DCP + 1.8) )
C
C---- fill 2-D J and Cp arrays required by CONPLT
      DO 32 K=1, NCP
        DO 31 L=1, NRJ
          CP(L,K) = CP0(K)
          RJ(L,K) = RJ0(L)
   31   CONTINUE
   32 CONTINUE
C
C---- find max and min beta, efficiency
      EFMAX = -1.0E9
      EFMIN = +1.0E9
      BEMAX = -1.0E9
      BEMIN = +1.0E9
      DO 30 L=1, NRJ
        DO 310 K=1, NCP
          IF(K.GE.KLOW(L) .AND. K.LE.KUPP(L)) THEN
           EFMAX = MAX( EFMAX , EF(L,K) )
           EFMIN = MIN( EFMIN , EF(L,K) )
           BEMAX = MAX( BEMAX , BE(L,K) )
           BEMIN = MIN( BEMIN , BE(L,K) )
          ENDIF
  310   CONTINUE
   30 CONTINUE
C
C
      CALL PLOPEN(SCRNFR,IPSLU,IDEV)
      CALL NEWFACTOR(SIZE)
      CALL PLOTABS(0.75,0.75,-3)
C
C---- set plot scale weights for J and Cp
      RJWT = 1.0/RJMAX
      CPWT = PAR/CPMAX
C
C---- J axis
      CALL NEWPEN(2)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
C
C---- tick marks
      NT = INT(RJMAX/DRJ)
      DO 51 IT=1, NT
        RJT = DRJ*FLOAT(IT)
        CALL PLOT(RJWT*RJT,-.2*CS,3)
        CALL PLOT(RJWT*RJT,0.2*CS,2)
        CALL PLNUMB(RJWT*RJT-1.5*CS,-2.0*CS,CS,RJT,0.0,1)
   51 CONTINUE
C
      CALL NEWPEN(3)
      XPLT = RJWT*DRJ*(FLOAT((3*NT)/4) + 0.5) - 0.6*CS
      YPLT = -3.0*CS
      CALL PLCHAR(XPLT,YPLT,1.3*CS,'J',0.0,1)
C
C---- Cp axis
      CALL NEWPEN(2)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(0.0,PAR,2)
C
C---- tick marks
      NT = INT(CPMAX/DCP)
      DO 53 IT=0, NT
        CPN = DCP*FLOAT(IT)
        CALL PLOT(-.2*CS,CPWT*CPN,3)
        CALL PLOT(0.2*CS,CPWT*CPN,2)
        CALL PLNUMB(-4.5*CS,CPWT*CPN-0.5*CS,CS,CPN,0.0,2)
   53 CONTINUE
C
      CALL NEWPEN(3)
      XPLT = -5.0*CS
      YPLT = CPWT*DCP*(FLOAT((3*NT)/4) + 0.5) - 0.6*CS
      CALL PLCHAR(XPLT       ,YPLT       ,1.3*CS,'C',0.0,1)
      CALL PLCHAR(XPLT+1.1*CS,YPLT-0.3*CS,1.0*CS,'p',0.0,1)
C
C---- plot propeller name in upper left corner of plot 
      CALL NEWPEN(3)
      XPLT = 1.5*CS
      YPLT = PAR  + 4.0*CS
C---- move title and specs up to clear the contour labels  HHY 11/98
      YPLT = YPLT + 3.5*CS
      CALL PLCHAR(XPLT,YPLT,1.4*CS,NAME,0.0,31)
C
C---- plot Mach PLNUMB and Reynolds PLNUMB
      YPLT = YPLT - 2.5*CS
      CALL PLCHAR(XPLT           ,YPLT,1.2*CS,'Mach/J = ',0.0,9)
      CALL PLNUMB(XPLT+9.0*1.2*CS,YPLT,1.2*CS,AMACH,0.0,3)
C
      XPLT = XPLT + 16.0*1.2*CS
      CALL PLCHAR(XPLT,YPLT,1.2*CS,'Re/J = ',0.0,7)
      CALL PLNUMB(XPLT+ 7.0*1.2*CS,YPLT       ,1.2*CS,RE6    ,0.0,3)
      CALL PLCHAR(XPLT+12.0*1.2*CS,YPLT       ,1.2*CS,'   10',0.0,5)
      CALL PLMATH(XPLT+12.0*1.2*CS,YPLT       ,1.2*CS,' #   ',0.0,5)
      CALL PLCHAR(XPLT+17.0*1.2*CS,YPLT+0.5*CS,0.9*CS,'6'    ,0.0,1)
C
      CALL PLFLUSH
C
      WRITE(*,*) ' '
      WRITE(*,*) 'Efficiency   min, max:', EFMIN, EFMAX
      WRITE(*,*) 'Blade angle  min, max:', BEMIN, BEMAX
      WRITE(*,*) ' '
C
  800 CONTINUE
      WRITE(*,*) ' '
      CALL ASKR('Enter lowest efficiency contour level^',EFLOW)
      CALL ASKR('Enter efficiency contour level increment^',DEF)
      CALL ASKI('Enter contour line thickness (1-5)^',LPEN)
      CALL ASKL('Add numerical labels to contours ?^',LABCON)
C
C**** plot contours of equal efficiency
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('GREEN')
      CALL NEWPEN(LPEN)
C
C---- go over efficiency contour levels
      DO 60 IEF = 0, 12345
C
C------ set efficiency contour level
        EFCON = EFLOW + DEF*FLOAT(IEF)
C
C------ skip out if outside upper limit
        IF(EFCON.GT.EFMAX) GO TO 61
C
        CALL CONPLT(LX,KX,NRJ,NCP,RJ,CP,KLOW,KUPP,
     &              EF,EFCON,0.0,0.0,RJWT,CPWT)
C
C------ draw label contours on right edge
        IF(LABCON) 
     &   CALL CONLAB(LX,KX,NRJ,NCP,RJ,CP,
     &               EF,EFCON,0.0,0.0,RJWT,CPWT,0.8*CS,2,2)
C
C------ draw label contours on top edge
        IF(LABCON) 
     &   CALL CONLAB(LX,KX,NRJ,NCP,RJ,CP,
     &               EF,EFCON,0.0,0.0,RJWT,CPWT,0.8*CS,2,3)
   60 CONTINUE
   61 CONTINUE
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      CALL ASKL('Add more efficiency contours ?^',YES)
      IF(YES) GO TO 800
C
C
  900 CONTINUE
      WRITE(*,*) ' '
      CALL ASKR('Enter largest blade angle contour level^',BEHIH)
      CALL ASKR('Enter blade angle contour level decrement^',DBE)
      CALL ASKI('Enter contour line thickness (1-5)^',LPEN)
      CALL ASKL('Add numerical labels to contours ?^',LABCON)
C
      DBE = ABS(DBE)
C
C**** plot contours of equal blade angle
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('RED')
      CALL NEWPEN(LPEN)
C
C---- go over blade angle contour levels
      NBE = INT(BEHIH/DBE) + 1
      DO 80 IBE = 0, NBE
C
        BECON = BEHIH - DBE*FLOAT(IBE)
C
C------ skip out if below lower limit
        IF(BECON.LT.BEMIN) GO TO 81
C
        CALL CONPLT(LX,KX,NRJ,NCP,RJ,CP,KLOW,KUPP,
     &              BE,BECON,0.0,0.0,RJWT,CPWT)
C
C------ draw contour labels on left edge
        IF(LABCON) 
     &   CALL CONLAB(LX,KX,NRJ,NCP,RJ,CP,
     &               BE,BECON,0.0,0.0,RJWT,CPWT,0.8*CS,-1,4)
   80 CONTINUE
   81 CONTINUE
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      CALL ASKL('Add more blade angle contours ?^',YES)
      IF(YES) GO TO 900
C
      CALL ASKL('Overlay grid ?^',YES)
      IF(YES) THEN
       CALL NEWPEN(1)
       X0 = RJWT*RJ(1,1)
       Y0 = CPWT*CP(1,1)
C
       DXG = RJWT*GRJ
       DYG = CPWT*GCP
       NXG = INT( (RJ(NRJ,NCP)-RJ(1,1))/GRJ + 0.01 )
       NYG = INT( (CP(NRJ,NCP)-CP(1,1))/GCP + 0.01 )
       CALL PLGRID(X0,Y0, NXG,DXG, NYG,DYG, -30584)
       CALL PLFLUSH
      ENDIF
C
      CALL ASKL('Hardcopy current plot ?^',YES)
      IF(YES) THEN
       CALL PLEND
       CALL ASKL('Color hardcopy plot ?^',YES)
       IF(YES) THEN
         CALL REPLOT(IDEVRPC)
        ELSE
         CALL REPLOT(IDEVRPB)
       ENDIF
      ENDIF
C
      CALL PLCLOSE
      ENDIF
C
      STOP
      END ! JPLOT


C--- alternate for Windoze DVFortran
c      SUBROUTINE GETFNAME(FNAME)
c      USE DFLIB
c      CHARACTER*80 FNAME
cC
c      CALL GETARG(1,FNAME)
c      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter dump filename^',FNAME)
cC 
c      RETURN
c      END
