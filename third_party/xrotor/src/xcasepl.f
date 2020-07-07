C***********************************************************************
C    Module:  xcasepl.f
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

      SUBROUTINE ACLPLT
      INCLUDE 'XROTOR.INC'
ccc      CHARACTER*4 DUMMY
C--------------------------------
C     Plots random stuff vs r/R
C--------------------------------
      EXTERNAL PLCHAR,PLMATH
      DIMENSION BETSAV(IX), BET0SAV(IX)
C
      DIMENSION XLIN(2), YLIN(2)
      DATA XLIN / 0.0, 1.0 /
      DATA YLIN / 0.0, 0.0 /
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      IF(NCASE.EQ.0) THEN
        WRITE(*,*) 'No cases saved'
        RETURN
      ENDIF
C
C---- x,y space between plot blocks
      XSPACE = 0.140
      YSPACE = 0.175
C
C---- legend line length
      DXLIN = 0.10
C
C---- plot scale factor and aspect ratio
      PLFAC = 0.45
      PLPAR = 1.15*PAR
C
C---- character size for axis numbers, labels
      CS  = CSIZE*1.3
      CSL = CSIZE*1.55
C
C---- CL-axis annotation limit and delta
      CLMAX1 = 2.0
      DCL = CLMAX1/5.0
C
C---- plotting scale factor for CL
      CLWT = PLPAR/CLMAX1

C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(0.80,0.7,-3)
C
      CALL GETCOLOR(ICOL0)
C
C---- CL(r/R) plot axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,  1.0,0.2     , 0.0,0.2, CS,1)
      CALL YAXIS(0.0,0.0,PLPAR,DCL*CLWT, 0.0,DCL, CS,1)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = 5
       CALL PLGRID(0.0,0.0, NXG,0.2, NYG,DCL*CLWT, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(0.7-1.5*CSL,-3.0*CSL,CSL,'r/R',0.0,3)
      CALL PLCHAR(-3.5*CSL,PLPAR-1.5*DCL*CLWT-0.5*CSL,1.2*CSL,'c',0.0,1)
      CALL PLSUBS(-3.5*CSL,PLPAR-1.5*DCL*CLWT-0.5*CSL,1.2*CSL,'V',0.0,1,
     &            PLMATH)
      IF(WIND)
     &CALL PLCHAR(-4.5*CSL,PLPAR-1.5*DCL*CLWT-0.5*CSL,1.2*CSL,'-',0.0,1)
C
C
C---- Gamma(r/R) plot axes
      YOFF = PLPAR + YSPACE
      CALL NEWPEN(2)
      CALL XAXIS(0.0,YOFF,  1.0,0.2      ,0.0,0.2,CS,1)
      CALL YAXIS(0.0,YOFF,PLPAR,0.2*PLPAR,0.0,0.2,CS,1)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = 5
       CALL PLGRID(0.0,YOFF, NXG,0.2, NYG,0.2*PLPAR, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(0.7-1.5*CSL,YOFF-3.0*CSL,CSL,'r/R',0.0,3)
C
      XPLT = -5.3*CSL
      YPLT = YOFF + PLPAR - 1.5*0.2*PLPAR + 0.2*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,'B /VR',0.0,5)
      CALL PLMATH(XPLT,YPLT,CSL,' G   ',0.0,5)
      YPLT = YPLT - 0.2*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,'_____',0.0,5)
      YPLT = YPLT - 1.6*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,' 2   ',0.0,5)
      CALL PLMATH(XPLT,YPLT,CSL,'  pl ',0.0,5)
      CALL PLSUBS(XPLT+3.0*CSL,YPLT,CSL,'w',0.0,1,PLCHAR)
C
C
C---- eta(r/R) plot axes
      XOFF = 1.0  + XSPACE
      YOFF = PLPAR + YSPACE
      CALL NEWPEN(2)
      CALL XAXIS(XOFF,YOFF, 1.0,0.2     ,0.0,0.2,CS,1)
      CALL YAXIS(XOFF,YOFF,PLPAR,0.2*PLPAR,0.0,0.2,CS,1)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = 5
       CALL PLGRID(XOFF,YOFF, NXG,0.2, NYG,0.2*PLPAR, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(XOFF+0.7-1.5*CSL,YOFF-3.0*CSL,CSL,'r/R',0.0,3)
      CALL PLMATH(XOFF-2.0*CSL,YOFF+PLPAR-1.5*0.2*PLPAR-0.5*CSL,
     &            1.2*CSL,'h'  ,0.0,1)
      IF(WIND)
     &CALL PLCHAR(XOFF-4.3*CSL,YOFF+PLPAR-1.5*0.2*PLPAR-0.5*CSL,
     &            1.2*CSL,'1/',0.0,2)
C
C
C---- save current parameters for restoration
      ADVSAV = ADV
      VELSAV = VEL
      ALTSAV = ALT
      RHOSAV = RHO
      VSOSAV = VSO
      RMUSAV = RMU
      DO I=1, II
        BETSAV(I) = BETA(I)
        BET0SAV(I) = BETA0(I)
      ENDDO
C
C
C---- location of case-list plot
      XLAB = 1.0 + XSPACE + DXLIN - 4.0*CSL
      YLAB = PLPAR - 4.0*CSL
C
      CALL NEWPEN(4)
      XPLT = XLAB
      YPLT = YLAB + 3.5*CSL
      CALL PLCHAR(XPLT,YPLT,1.3*CSL,NAME,0.0,31)
C
C---- case-list label
      CALL NEWPEN(3)
      XPLT = XLAB
      YPLT = YLAB + 0.5*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,
     & '      V/ R            V      alt     T        ',0.0,46)
      CALL PLMATH(XPLT,YPLT,CSL,
     & '        W     b                              h',0.0,46)
      IF(WIND) CALL PLCHAR(XPLT+43.0*CSL,YPLT,CSL,'1/',0.0,2)
      CALL PLCHAR(XPLT+15.0*CSL,YPLT-0.3*CSL,0.8*CSL,'tip',0.0,3)
      CALL PLCHAR(XPLT+38.0*CSL,YPLT-0.3*CSL,    CSL,'c'  ,0.0,1)
CCC      10  0.1234  42.34  123.12  15.00  0.4321  0.872
CCC     1234567890123456789012345678901234567890123456789012
CCC              1         2         3         4         5
C
C
C---- calculate cases, and plot each one
      DO 100 ICASE=1, NCASE
C
      IF(NCASE.GT.8) THEN
C------ for .gt.8 cases, use spectrum colors
        FRAC = FLOAT(ICASE-1)/FLOAT(NCASE-1)
        ICOL = 1 + INT( FLOAT(NCOLOR-1)*(FRAC + 0.01) )
        ICOL = -ICOL
      ELSE
C------ for 8 or less cases, use default colors
        ICOL = ICASE+2
      ENDIF
C
      CALL NEWCOLOR(ICOL)
C
C---- set parameters for this case
      ITQP = IFIX(CASPAR(0,ICASE)/1000.0)
      IPWR = IFIX(CASPAR(0,ICASE)/100.0) - 10*ITQP
      ITYP = IFIX(CASPAR(0,ICASE)) - 100*IPWR - 1000*ITQP
      ADV = CASPAR(1,ICASE)
      VEL = CASPAR(2,ICASE)
      BET = CASPAR(3,ICASE)
      ALT = CASPAR(4,ICASE)
      RHO = CASPAR(5,ICASE)
      RMU = CASPAR(6,ICASE)
      VSO = CASPAR(7,ICASE)
C
      DELB  = BET - BETSAV(II)
      DO I=1, II
        BETA(I)  = BETSAV(I)  + DELB
        BETA0(I) = BET0SAV(I) + DELB
      END DO
C
C---- converge this case
      CONV = .FALSE.
      IF(IPWR.EQ.0) THEN
C
        IF(ITYP.EQ.1) THEN
          CALL APER(4,2,.TRUE.)
C
         ELSEIF(ITYP.EQ.2) THEN
          IF(ITQP.EQ.1) THEN
           CALL APER(1,1,.TRUE.)
          ELSEIF(ITQP.EQ.2) THEN
           CALL APER(2,1,.TRUE.)
          ELSEIF(ITQP.EQ.3) THEN
           CALL APER(3,1,.TRUE.)
          ELSE
           CALL APER(4,2,.TRUE.)
          ENDIF
C
         ELSEIF(ITYP.EQ.3) THEN
          CALL APER(4,2,.TRUE.)
C
         ELSEIF(ITYP.EQ.4) THEN
          CALL APER(4,2,.TRUE.)
C
         ELSEIF(ITYP.EQ.5) THEN
          CALL APER(3,1,.TRUE.)
C
        ENDIF
       ELSE
        IF(ITYP.EQ.1) THEN
          CALL APER(5,1,.TRUE.)
         ELSEIF(ITYP.EQ.2) THEN
          CALL APER(5,1,.TRUE.)
         ELSEIF(ITYP.EQ.3) THEN
          CALL APER(5,2,.TRUE.)
         ELSEIF(ITYP.EQ.4) THEN
          CALL APER(5,2,.TRUE.)
         ELSEIF(ITYP.EQ.5) THEN
          CALL APER(5,1,.TRUE.)
        ENDIF
      ENDIF
C
ccc       RPM = VEL/(ADV*RAD) * 30.0/PI
      CASPAR(1,ICASE) = ADV
      CASPAR(3,ICASE) = BETA(II)
      IF(CONV) THEN
       CASPAR(8,ICASE) = PTOT*RHO*VEL**3*RAD**2
       CASPAR(9,ICASE) = TTOT*RHO*VEL**2*RAD**2
       CASPAR(10,ICASE)= QTOT*RHO*VEL**2*RAD**3
       CASPAR(11,ICASE)= TTOT/PTOT
      ELSE
       CASPAR(8,ICASE) = 999.
       CASPAR(9,ICASE) = 999.
       CASPAR(10,ICASE)= 999.
       CASPAR(11,ICASE)= 999.
      ENDIF
C
C---- plot CL vs r/R
      CALL NEWPEN(3)
      XOFF = 0.0
      YOFF = 0.0
C
      YOFFS = YOFF
      CLWTS = CLWT
      IF(WIND) THEN
       YOFFS = -YOFF
       CLWTS = -CLWT
      ENDIF
      CALL XYLINE(II,XI,CL,XOFF,1.0,YOFFS,CLWTS,ICASE)
C
C---- plot normalized Gamma vs r/R
      CALL NEWPEN(3)
ccc      CALL SCALIT(II,GAM,0.0,GFAC)
      EFFINV = PWAK/TWAK
      ZETA = 2.0*(EFFINV-1.0)
      GFAC = 0.5*FLOAT(NBLDS)/(PI*ADV*ZETA*EFFINV)
      XOFF = 0.0
      YOFF = -(PLPAR + YSPACE)/(GFAC*PLPAR)
      CALL XYLINE(II,XI,GAM,XOFF,1.0,YOFF,GFAC*PLPAR,ICASE)
C
C---- plot local efficiency vs r/R
      CALL NEWPEN(3)
      DO I=1, II
C------ use equivalent prop to define efficiency
        VW = VWAK(I)
        UTOTW = URDUCT
        CALL UVADD(XI(I),WA,WT)
C
        CW = XI(I)/ADV - WT  -  VW
        SW = UTOTW     + WA  +  VW*XW(I)/ADW
C
        EFFI = (CW/SW) * ADV/XW(I)
C
        W1(I) = EFFI*EFFP(I)
        IF(WIND) W1(I) = 1.0/W1(I)
C
        W1(I) = MAX( W1(I) , 0.0 )
      ENDDO
      XOFF = -( 1.0 + XSPACE)
      YOFF = -(PLPAR + YSPACE)/PLPAR
      CALL XYLINE(II,XI,W1,XOFF,1.0,YOFF,PLPAR,ICASE)
C
C
C---- plot case parameters on legend
      RCASE = FLOAT(ICASE)
C
      XPLT = XLAB
      YPLT = YLAB - FLOAT(ICASE)*2.0*CSL
C
      CALL NEWPEN(3)
      XOFF = -(XPLT- DXLIN )/DXLIN
      YOFF = -(YPLT+0.6*CSL)/DXLIN
      CALL XYLINE(2,XLIN,YLIN,XOFF,DXLIN,YOFF,DXLIN,ICASE)
C
      XPLT = XPLT + 2.0*CSL
      DELX = 0.0
      IF(ICASE.GT. 9) DELX = DELX - CSL
      IF(ICASE.GT.99) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,RCASE,0.0,-1)
C
      XPLT = XPLT + 3.0*CSL
      DELX = 0.0
      IF(    ADV  .LT.  0.0) DELX = DELX - CSL
      IF(ABS(ADV) .GE. 10.0) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,ADV,0.0,4)
C
      XPLT = XPLT + 9.0*CSL
      DELX = 0.0
      BDTIP = BETA(II)*180.0/PI
      IF(    BDTIP  .LT.   0.0) DELX = DELX - CSL
      IF(ABS(BDTIP) .GE.  10.0) DELX = DELX - CSL
      IF(ABS(BDTIP) .GE. 100.0) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,BDTIP,0.0,2)
C
      XPLT = XPLT + 8.0*CSL
      DELX = 0.0
      IF(VEL .GE.  10.0) DELX = DELX - CSL
      IF(VEL .GE. 100.0) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,VEL,0.0,2)
C
      XPLT = XPLT + 7.0*CSL
      IF(ALT .EQ. 999.0) THEN
       CALL PLCHAR(XPLT,YPLT,CSL,' - ',0.0,3)
      ELSE
       DELX = 0.0
       IF(ALT .LT.   0.0) DELX = DELX - CSL
       IF(ALT .GE.  10.0) DELX = DELX - CSL
       IF(ALT .GE. 100.0) DELX = DELX - CSL
       CALL PLNUMB(XPLT+DELX,YPLT,CSL,ALT,0.0,2)
      ENDIF
C
      XPLT = XPLT + 6.0*CSL
      DELX = 0.0
      TC = TTOT*2.0/PI
      IF(TC .LT.   0.0) DELX = DELX - CSL
      IF(TC .GE.  10.0) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,TC,0.0,4)
C
      XPLT = XPLT + 7.0*CSL
      DELX = 0.0
      EFF = TTOT/PTOT
      IF(WIND) EFF = 1.0/EFF
      IF(    EFF  .LT.   0.0) DELX = DELX - CSL
      IF(ABS(EFF) .GE.  10.0) DELX = DELX - CSL
      CALL PLNUMB(XPLT+DELX,YPLT,CSL,EFF,0.0,3)
C
 100  CONTINUE
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
C---- restore saved operating condition
      ADV = ADVSAV
      VEL = VELSAV
      ALT = ALTSAV
      RHO = RHOSAV
      VSO = VSOSAV
      RMU = RMUSAV
      DO I=1, II
        BETA(I) = BETSAV(I)
        BETA0(I) = BET0SAV(I)
      ENDDO
C
      CONV = .FALSE.
      CALL APER(4,2,.TRUE.)
C
      RETURN
      END ! ACLPLT




      SUBROUTINE CASPLT
      INCLUDE 'XROTOR.INC'
C---------------------------------------------------
C     Plots operating parameters over a range 
C     of advance ratio, rpm, or beta
C---------------------------------------------------
      DIMENSION BETSAV(IX), BET0SAV(IX)
      EXTERNAL PLCHAR,PLMATH
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      IF(NCASE.EQ.0) THEN
        WRITE(*,*) 'No cases saved'
        RETURN
      ENDIF
C
C---- plot scale factor and aspect ratio
      PLFAC = 0.8
      PLPAR = PAR
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.2
C
      ITYPE = KCASE
      IF(ITYPE.LE.0) THEN
        WRITE(*,*)
        WRITE(*,*) '  1  advance ratio'
        WRITE(*,*) '  2  rpm'
        WRITE(*,*) '  3  blade angle'
        WRITE(*,*) '  4  velocity with blade angle fixed'
        WRITE(*,*) '  5  velocity with blade angle varying'
        CALL ASKI('Enter case-sweep parameter^',ITYPE)
      ENDIF
C
C---- save current advance ratio and angles for restoration
      ADVSAV = ADV
      VELSAV = VEL
      ALTSAV = ALT
      RHOSAV = RHO
      VSOSAV = VSO
      RMUSAV = RMU
      DO I=1, II
        BETSAV(I)  = BETA(I)
        BET0SAV(I) = BETA0(I)
      ENDDO
C
C
C---- calculate each case and plot it
      DO 10 ICASE=1, NCASE
C---- set parameters for this case
        ITQP = IFIX(CASPAR(0,ICASE)/1000.0)
        IPWR = IFIX(CASPAR(0,ICASE)/100.0) - 10*ITQP
        ITYP = IFIX(CASPAR(0,ICASE)) - 100*IPWR - 1000*ITQP
        ADV = CASPAR(1,ICASE)
        VEL = CASPAR(2,ICASE)
        BET = CASPAR(3,ICASE)
        ALT = CASPAR(4,ICASE)
        RHO = CASPAR(5,ICASE)
        RMU = CASPAR(6,ICASE)
        VSO = CASPAR(7,ICASE)
C
        DELB = BET - BETSAV(II)
        DO I=1, II
          BETA(I)  = BETSAV(I)  + DELB
          BETA0(I) = BET0SAV(I) + DELB
        ENDDO
C
        CONV = .FALSE.
        IF(IPWR.EQ.0) THEN
          IF(ITYP.NE.5) THEN
            CALL APER(4,2,.TRUE.)
           ELSE
            CALL APER(3,1,.TRUE.)
          ENDIF
C
          IF(ITYP.EQ.1) THEN
            CALL APER(4,2,.TRUE.)
C
          ELSEIF(ITYP.EQ.2) THEN
            IF(ITQP.EQ.1) THEN
             CALL APER(1,1,.TRUE.)
            ELSEIF(ITQP.EQ.2) THEN
             CALL APER(2,1,.TRUE.)
            ELSEIF(ITQP.EQ.3) THEN
             CALL APER(3,1,.TRUE.)
            ELSE
             CALL APER(4,2,.TRUE.)
            ENDIF
C
          ELSEIF(ITYP.EQ.3) THEN
           CALL APER(4,2,.TRUE.)
C
          ELSEIF(ITYP.EQ.4) THEN
           CALL APER(4,2,.TRUE.)
C
          ELSEIF(ITYP.EQ.5) THEN
           CALL APER(3,1,.TRUE.)
          ENDIF

         ELSE
          IF(ITYP.EQ.1) THEN
             CALL APER(5,1,.TRUE.)
           ELSEIF(ITYP.EQ.2) THEN
             CALL APER(5,1,.TRUE.)
           ELSEIF(ITYP.EQ.3) THEN
             CALL APER(5,2,.TRUE.)
           ELSEIF(ITYP.EQ.4) THEN
             CALL APER(5,2,.TRUE.)
           ELSEIF(ITYP.EQ.5) THEN
             CALL APER(5,1,.TRUE.)
          ENDIF
        ENDIF
C
ccc     RPM = VEL/(ADV*RAD) * 30.0/PI
        CASPAR(1,ICASE) = ADV
        CASPAR(3,ICASE) = BETA(II)
        IF(CONV) THEN
         CASPAR(8,ICASE) = PTOT*RHO*VEL**3*RAD**2
         CASPAR(9,ICASE) = TTOT*RHO*VEL**2*RAD**2
         CASPAR(10,ICASE)= QTOT*RHO*VEL**2*RAD**3
         CASPAR(11,ICASE)= TTOT/PTOT
        ELSE
         CASPAR(8,ICASE) = 999.
         CASPAR(9,ICASE) = 999.
         CASPAR(10,ICASE)= 999.
         CASPAR(11,ICASE)= 999.
        ENDIF
C
C------ fill temporary arrays with operating parameters
        IF    (ITYP.EQ.1) THEN
          W1(ICASE) = ADV
        ELSEIF(ITYP.EQ.2) THEN
          W1(ICASE) = VEL/(ADV*RAD) * 30.0/PI
        ELSEIF(ITYP.EQ.3) THEN
          W1(ICASE) = BETA(II) * 180.0/PI
        ELSEIF(ITYP.EQ.4) THEN
          W1(ICASE) = VEL
        ELSEIF(ITYP.EQ.5) THEN
          W1(ICASE) = VEL
        ENDIF
C
        IF(LVNORM) THEN
         VNORM = 1.0
        ELSE
         VNORM = 1.0/ADV
        ENDIF
C
        IF(WIND) THEN
C------- windmill definitions of Tc, Pc, efficiency
         W2(ICASE) = -TTOT * 2.0/PI / VNORM**2
         W3(ICASE) = -PTOT * 2.0/PI / VNORM**3
         W4(ICASE) = PTOT/TTOT
        ELSE
C------- propeller definitions of Tc, Pc, efficiency
         W2(ICASE) =  TTOT * 2.0/PI / VNORM**2
         W3(ICASE) =  PTOT * 2.0/PI / VNORM**3
         W4(ICASE) = TTOT/PTOT
        ENDIF
   10 CONTINUE
C
      XMIN = W1(1)
      XMAX = W1(1)
      DO N=1, NCASE
        XMIN = MIN(XMIN,W1(N))
        XMAX = MAX(XMAX,W1(N))
      ENDDO
      IF(XMIN.EQ.XMAX) XMAX = XMIN + 1.0
C
      CALL SCALIT(1,XMAX,XMIN,XSF1)
      XDEL = 1.0/(5.0*XSF1)
C
      XMIN = XDEL * (AINT(XMIN/XDEL - 1000.99) + 1000.0)
      XMAX = XDEL * (AINT(XMAX/XDEL + 1000.99) - 1000.0)
      XSF = 1.0/(XMAX-XMIN)
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(1.25,1.00,-3)
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,1.0,XDEL*XSF,XMIN,XDEL,CS,-2)
C
      CALL NEWPEN(3)
      XLAB = (XMAX-1.5*XDEL - XMIN)*XSF
      IF    (ITYP.EQ.1) THEN
       CALL PLCHAR(XLAB-2.0*CSL,-3.0*CSL,CSL,'V/ R',0.0,4)
       CALL PLMATH(XLAB-2.0*CSL,-3.0*CSL,CSL,'  W ',0.0,4)
      ELSEIF(ITYP.EQ.2) THEN
       CALL PLCHAR(XLAB-1.5*CSL,-3.0*CSL,CSL,'rpm',0.0,3)
      ELSEIF(ITYP.EQ.3) THEN
       CALL PLMATH(XLAB-0.9*CSL,-3.0*CSL,CSL,'b',0.0,1)
       CALL PLSUBS(XLAB-0.9*CSL,-3.0*CSL,CSL,'tip',0.0,3,PLCHAR)
       CALL PLMATH(XLAB+0.2*CSL,-3.0*CSL,CSL,'"',0.0,1)
      ELSEIF(ITYP.EQ.4) THEN
       CALL PLCHAR(XLAB-0.5*CSL,-3.0*CSL,CSL,'V',0.0,1)
      ELSEIF(ITYP.EQ.5) THEN
       CALL PLCHAR(XLAB-0.5*CSL,-3.0*CSL,CSL,'V',0.0,1)
      ENDIF
C
      CALL SCALIT(NCASE,W3,0.0,PFAC)
      PMAX = 1.0/PFAC
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,0.0,PLPAR,PLPAR/5.0,0.0,PMAX/5.0, CS,-2)
      CALL YAXIS(1.0,0.0,PLPAR,PLPAR/5.0,0.0,0.2     ,-CS,1)
C
      YLAB = PLPAR + 8.0*CSL
C
      CALL NEWPEN(4)
      CALL PLCHAR(0.0,YLAB+0.4*CSL,1.2*CSL,NAME,0.0,31)
C
      YL = YLAB
      CALL NEWPEN(3)
C
      IF(ITYP.NE.1 .AND. ITYP.NE.2 .AND. 
     &   ITYP.NE.4 .AND. ITYP.NE.5) THEN
       YL = YL - 2.2*CSL
       CALL PLCHAR(0.0 ,YL,CSL,'V/ R = ',0.0,7)
       CALL PLMATH(0.0 ,YL,CSL,'  W    ',0.0,7)
       CALL PLNUMB(999.,YL,CSL, ADV     ,0.0,4)
      ENDIF
      IF(ITYP.NE.1 .AND. ITYP.NE.2) THEN
       YL = YL - 2.2*CSL
       RPM = VEL/(ADV*RAD) * 30.0/PI
       CALL PLCHAR(0.0 ,YL,CSL,'rpm  = ',0.0,7)
       CALL PLNUMB(999.,YL,CSL, RPM     ,0.0,1)
      ENDIF
      IF(ITYP.NE.3) THEN
       YL = YL - 2.2*CSL
       CALL PLSUBS( 0.0,YL,CSL,'tip'    ,0.0,3,PLCHAR)
       CALL PLMATH( 0.0,YL,CSL,'b    = ',0.0,7)
       CALL PLNUMB(999.,YL,CSL, BETA(II)*180.0/PI,0.0,3)
      ENDIF
      IF(ITYP.NE.4 .AND. ITYP.NE.5) THEN
       YL = YL - 2.2*CSL
       CALL PLCHAR(0.0 ,YL,CSL,'V    = ',0.0,7)
       CALL PLNUMB(999.,YL,CSL, VEL     ,0.0,2)
      ENDIF
C
      XL = 22.0*CSL
      YL = YLAB
      CALL NEWPEN(3)
C
      YL = YL - 2.2*CSL
      CALL PLMATH(XL  ,YL,CSL,'r  = ',0.0,5)
      CALL PLNUMB(999.,YL,CSL, RHO   ,0.0,4)
C
      YL = YL - 2.2*CSL
      CALL PLCHAR(XL  ,YL,CSL,'R  = ',0.0,5)
      CALL PLNUMB(999.,YL,CSL, RAD   ,0.0,4)
C
C
C
      CALL NEWPEN(3)
      YSF = PLPAR*PFAC
C
      CALL NEWCOLORNAME('orange')
      IF(WIND)
     &CALL PLCHAR(-6.2*CSL,0.7*PLPAR        ,CSL,' -    ',0.0,6)
      CALL PLCHAR(-6.2*CSL,0.7*PLPAR        ,CSL,'  2T  ',0.0,6)
      CALL PLCHAR(-6.2*CSL,0.7*PLPAR        ,CSL,'______',0.0,6)
      IF(LVNORM) THEN
       CALL PLCHAR(-6.2*CSL,0.7*PLPAR-1.9*CSL,CSL,' V  R ',0.0,6)
       CALL PLMATH(-6.2*CSL,0.7*PLPAR-1.9*CSL,CSL,'r 2p 2',0.0,6)
      ELSE
       CALL PLCHAR(-6.2*CSL,0.7*PLPAR-1.9*CSL,CSL,'    R ',0.0,6)
       CALL PLMATH(-6.2*CSL,0.7*PLPAR-1.9*CSL,CSL,'rW2p 4',0.0,6)
      ENDIF
      CALL XYLINE(NCASE,W1,W2,XMIN,XSF,0.0,YSF,2)
C
      CALL NEWCOLORNAME('red')
      IF(WIND)
     &CALL PLCHAR(-6.2*CSL,0.9*PLPAR        ,CSL,' -    ',0.0,6)
      CALL PLCHAR(-6.2*CSL,0.9*PLPAR        ,CSL,'  2P  ',0.0,6)
      CALL PLCHAR(-6.2*CSL,0.9*PLPAR        ,CSL,'______',0.0,6)
      IF(LVNORM) THEN
       CALL PLCHAR(-6.2*CSL,0.9*PLPAR-1.9*CSL,CSL,' V  R ',0.0,6)
       CALL PLMATH(-6.2*CSL,0.9*PLPAR-1.9*CSL,CSL,'r 3p 2',0.0,6)
      ELSE
       CALL PLCHAR(-6.2*CSL,0.9*PLPAR-1.9*CSL,CSL,'    R ',0.0,6)
       CALL PLMATH(-6.2*CSL,0.9*PLPAR-1.9*CSL,CSL,'rW3p 5',0.0,6)
      ENDIF
      CALL XYLINE(NCASE,W1,W3,XMIN,XSF,0.0,YSF,3)
C
      CALL NEWCOLORNAME('cyan')
      IF(WIND) THEN
       CALL PLCHAR(1.0+1.5*CSL,0.9*PLPAR-0.5*CSL,1.2*CSL,'1/ ',0.0,3)
       CALL PLMATH(1.0+1.5*CSL,0.9*PLPAR-0.5*CSL,1.2*CSL,'  h',0.0,3)
      ELSE
       CALL PLMATH(1.0+2.5*CSL,0.9*PLPAR-0.5*CSL,1.2*CSL,'h',0.0,1)
      ENDIF
      CALL XYLINE(NCASE,W1,W4,XMIN,XSF,0.0,PLPAR,1)
C
      CALL PLFLUSH
      CALL NEWCOLOR(ICOL0)
C
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT( (XMAX-XMIN)/(0.5*XDEL) + 0.01 )
       NYG = 10
       CALL PLGRID(0.0,0.0, NXG,XSF*0.5*XDEL, NYG,0.1*PLPAR, LMASK2 )
      ENDIF
C
      CALL PLFLUSH
C
C---- restore previous operating point
      ADV = ADVSAV
      VEL = VELSAV
      ALT = ALTSAV
      VSO = VSOSAV
      RHO = RHOSAV
      RMU = RMUSAV
      DO I=1, II
        BETA(I) = BETSAV(I)
        BETA0(I) = BET0SAV(I)
      ENDDO
      CONV = .FALSE.
      CALL APER(4,2,.TRUE.)
C
      RETURN
      END ! CASPLT

