C***********************************************************************
C    Module:  xrotpl.f
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

C    ******************************************************
C    *     All the graphics subroutines below require     *
C    *     Xplot11 plotting software support              *
C    ******************************************************


      SUBROUTINE GEOPLT(VIEW)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) VIEW
C------------------------------------------
C     Plots blade planform and projections
C------------------------------------------
      DIMENSION XLIN(4), YLIN(4)
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.4
C
C---- case title
      CALL NEWPEN(4)
      XT = 0.0
      YT = -0.030
      CALL PLCHAR(XT,YT,CSL,NAME,0.0,-1)
C
C---- vertical space/R between projections
      YSPACE = 0.375
C
      CHUMAX = 0.
      CHLMAX = 0.
      DO 10 I=1, II
        SINB = SIN(BETA(I))
        COSB = COS(BETA(I))
C
C------ projection chords in front and behind radial axis
        W1(I) =  XPITCH     *SINB*CH(I)
        W2(I) = (XPITCH-1.0)*SINB*CH(I)
C
        W3(I) =  XPITCH     *COSB*CH(I)
        W4(I) = (XPITCH-1.0)*COSB*CH(I)
C
        W5(I) =  XPITCH     *CH(I)
        W6(I) = (XPITCH-1.0)*CH(I)
C
        CHUMAX = MAX(CHUMAX,     CH(I))
        CHLMAX = MAX(CHLMAX,SINB*CH(I))
   10 CONTINUE
C
C---- add vertical space for wide chords
ccc      YSPACE = MAX( YSPACE , 1.05*CHUMAX )
C
C---- plot untwisted view if selected
      IVIEW = INDEX(VIEW,'U') + INDEX(VIEW,'u')
      IF(IVIEW.GT.0) THEN
        CALL PLOT(0.0,(FLOAT(IVIEW)-0.5)*YSPACE,-3)
        CALL NEWPEN(1)
C
C------ plot radial axis
        CALL PLOT(0.0,0.0,3)
        CALL PLOT(1.0,0.0,2)
C
C------ plot radial tick marks
        DO NT=0, 5
          XT = 0.2*FLOAT(NT)
          CALL PLOT(XT,-.005,3)
          CALL PLOT(XT,0.005,2)
        ENDDO
C
C------ plot blade shape
        CALL NEWPEN(3)
        CALL XYLINE(II,XI,W5,0.0,1.0,0.0,1.0,1)
        CALL XYLINE(II,XI,W6,0.0,1.0,0.0,1.0,1)
C
C------ plot hub line
        CALL NEWPEN(2)
        CALL PLOT(XI0,CH(1)* XPITCH     ,3)
        CALL PLOT(XI0,CH(1)*(XPITCH-1.0),2)
C
C---- label for plot view
        XT = XI(1) + 1.0*CS
        YT = -0.35*YSPACE
        CALL PLCHAR(XT,YT,CS,'Untwisted',0.0,-1)
C
        CALL PLOT(0.0,-(FLOAT(IVIEW)-0.5)*YSPACE,-3)
      ENDIF
C
C
C---- plot lateral view if selected
      IVIEW = INDEX(VIEW,'L') + INDEX(VIEW,'l')
      IF(IVIEW.GT.0) THEN
        CALL PLOT(0.0,(FLOAT(IVIEW)-0.5)*YSPACE,-3)
        CALL NEWPEN(1)
C
C------ plot radial axis
        CALL PLOT(0.0,0.0,3)
        CALL PLOT(1.0,0.0,2)
C
C------ plot rotation axis
        CALL PLOT(0.0, 0.1,3)
        CALL PLOT(0.0,-0.2,2)
C
C------ plot radial tick marks
        DO NT=0, 5
          XT = 0.2*FLOAT(NT)
          CALL PLOT(XT,-.005,3)
          CALL PLOT(XT,0.005,2)
        ENDDO
C
C------ plot blade shape
        CALL NEWPEN(3)
        CALL XYLINE(II,XI,W1,0.0,1.0,0.0,1.0,1)
        CALL XYLINE(II,XI,W2,0.0,1.0,0.0,1.0,1)
C
C------ plot hub surface
        CALL NEWPEN(2)
        XLIN(1) = XI0
        XLIN(2) = XI0
        XLIN(3) = XW0
        XLIN(4) = XW0
        YLIN(1) = CHLMAX* XPITCH     
        YLIN(2) = CHLMAX*(XPITCH-1.0)
        YLIN(3) = YLIN(2) - 0.05
        YLIN(4) = YLIN(3) - 0.05
C
        CALL PLOT(XLIN(1),YLIN(1),3)
        CALL PLOT(XLIN(2),YLIN(2),2)
C
        CALL XYLINE(2,XLIN(2),YLIN(2),0.0,1.0,0.0,1.0,4)
        CALL XYLINE(2,XLIN(3),YLIN(3),0.0,1.0,0.0,1.0,3)
C
C---- label for plot view
        XT = XI(1) + 1.0*CS
        YT = -0.35*YSPACE
        CALL PLCHAR(XT,YT,CS,'Lateral',0.0,-1)
C
        CALL PLOT(0.0,-(FLOAT(IVIEW)-0.5)*YSPACE,-3)
      ENDIF
C
C
C---- plot axial view if selected
      IVIEW = INDEX(VIEW,'A') + INDEX(VIEW,'a')
      IF(IVIEW.GT.0) THEN
        CALL PLOT(0.0,(FLOAT(IVIEW)-0.5)*YSPACE,-3)
        CALL NEWPEN(1)
C
C------ plot radial axis
        CALL PLOT(0.0,0.0,3)
        CALL PLOT(1.0,0.0,2)
C
C------ for axial view, plot blade "spokes" (A)
        RSPOKE = MAX( 0.1 , 1.25*XI0 )
        DO IB=2, NBLDS
          ANG = 2.0*PI * FLOAT(IB-1)/FLOAT(NBLDS)
          XP = RSPOKE*COS(ANG)
          YP = RSPOKE*SIN(ANG)
          CALL PLOT(0.0,0.0,3)
          CALL PLOT(XP,YP,2)
        ENDDO
C
C------ plot radial tick marks
        DO NT=0, 5
          XT = 0.2*FLOAT(NT)
          CALL PLOT(XT,-.005,3)
          CALL PLOT(XT,0.005,2)
        ENDDO
C
C------ plot hub circle
        CALL NEWPEN(2)
        CALL PLCIRC(0.0,0.0,XI0,0)
C
C------ plot wake circle
        CALL NEWPEN(2)
        CALL PLCIRC(0.0,0.0,XW0,18)
C
C------ plot blade shape
        CALL NEWPEN(3)
        CALL XYLINE(II,XI,W3,0.0,1.0,0.0,1.0,1)
        CALL XYLINE(II,XI,W4,0.0,1.0,0.0,1.0,1)
C
C---- label for plot view
        XT = XI(1) + 1.0*CS
        YT = -0.35*YSPACE
        CALL PLCHAR(XT,YT,CS,'Axial',0.0,-1)
C
        CALL PLOT(0.0,-(FLOAT(IVIEW)-0.5)*YSPACE,-3)
      ENDIF
C
C
C---- plot end view if selected
      KVIEW = INDEX(VIEW,'E') + INDEX(VIEW,'e')
      IVIEW = INDEX(VIEW,'L') + INDEX(VIEW,'l')
      IF(KVIEW.GT.0 .AND. IVIEW.GT.0) THEN
C
        DXEND = 1.25
        CALL PLOT(DXEND,(FLOAT(IVIEW)-0.5)*YSPACE,-3)
C
C------ plot blade shape
        CALL NEWPEN(3)
        CALL XYLINE(II,W3,W1,0.0,1.0,0.0,1.0,1)
        CALL XYLINE(II,W4,W2,0.0,1.0,0.0,1.0,1)
C
        I = 1
        CALL PLOT(W3(I),W1(I),3)
        CALL PLOT(W4(I),W2(I),2)
C
        I = II
        CALL PLOT(W3(I),W1(I),3)
        CALL PLOT(W4(I),W2(I),2)
C
C------ plot chord lines
        CALL NEWPEN(1)
        DO NT=1, 5
          XT = 0.2*FLOAT(NT)
          DO I=1, II-1
            IF(XT.GT.XI(I) .AND. XT.LE.XI(I+1)) THEN
             FRAC = (XT-XI(I)) / (XI(I+1)-XI(I))
             W1T = W1(I) + FRAC*(W1(I+1)-W1(I))
             W2T = W2(I) + FRAC*(W2(I+1)-W2(I))
             W3T = W3(I) + FRAC*(W3(I+1)-W3(I))
             W4T = W4(I) + FRAC*(W4(I+1)-W4(I))
             CALL PLOT(W3T,W1T,3)
             CALL PLOT(W4T,W2T,2)
            ENDIF
          ENDDO
        ENDDO
C
C------ plot rotation axis
        CALL PLOT(0.0, 0.1,3)
        CALL PLOT(0.0,-0.2,2)
C
C------ plot hub surface
        CALL NEWPEN(2)
        XLIN(1) = XI0
        XLIN(2) = XI0
        XLIN(3) = XW0
        XLIN(4) = XW0
        YLIN(1) = CHLMAX* XPITCH     
        YLIN(2) = CHLMAX*(XPITCH-1.0)
        YLIN(3) = YLIN(2) - 0.05
        YLIN(4) = YLIN(3) - 0.05
C
        CALL PLOT( XLIN(1),YLIN(1),3)
        CALL PLOT( XLIN(2),YLIN(2),2)
        CALL PLOT(-XLIN(1),YLIN(1),3)
        CALL PLOT(-XLIN(2),YLIN(2),2)
C
        CALL XYLINE(2,XLIN(2),YLIN(2),0.0, 1.0,0.0,1.0,4)
        CALL XYLINE(2,XLIN(3),YLIN(3),0.0, 1.0,0.0,1.0,3)
        CALL XYLINE(2,XLIN(2),YLIN(2),0.0,-1.0,0.0,1.0,4)
        CALL XYLINE(2,XLIN(3),YLIN(3),0.0,-1.0,0.0,1.0,3)
C
C------ plot prop-plane line
        DS = 0.2
        CALL NEWPEN(1)
        CALL NEWPAT(LMASK2)
        CALL PLOT(-DS,0.0,3)
        CALL PLOT( DS,0.0,2)
C
        CALL NEWPAT(LMASK1)
C
C------ plot hub lines
        DX = DS*COS(BETA(1))
        DY = DS*SIN(BETA(1))
        CALL PLOT(-DX,-DY,3)
        CALL PLOT( DX, DY,2)
C
C------ and tip-angle lines
        DX = DS*COS(BETA(II))
        DY = DS*SIN(BETA(II))
        CALL PLOT(-DX,-DY,3)
        CALL PLOT( DX, DY,2)
C
C---- label for plot view
        XT = -1.5*CS
        YT = -0.2 - 2.0*CS
        CALL PLCHAR(XT,YT,CS,'End',0.0,-1)
C
        CALL PLOT(-DXEND,-(FLOAT(IVIEW)-0.5)*YSPACE,-3)
      ENDIF
C
      CALL PLFLUSH
      RETURN
      END ! GEOPLT



      SUBROUTINE CLPLT
      INCLUDE 'XROTOR.INC'
C---------------------------------------------------
C     Plots CL, eta, normalized Gamma vs r/R
C     Modified 10/21/00 for helicopter work at Frontier Systems
C     by HHY
C---------------------------------------------------
      LOGICAL LMAPLT, LHELI
C
      EXTERNAL PLCHAR,PLMATH
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      DTR = ATAN(1.0)/45.0
      RTD = 1.0/DTR
C
      LHELI = .FALSE.
C---- plot aspect ratio
      PLPAR = PAR
C
C---- character size for axis numbers, labels
      CS  = 0.9*CSIZE
      CSL = CS*1.4
C
C---- CL-axis limit and increment
      CLMAX1 = 2.0
      DCL = CLMAX1/5.0
C
C---- dimensional power, thrust
      PDIM = PTOT*RHO*VEL**3*RAD**2
      TDIM = TTOT*RHO*VEL**2*RAD**2
C
C---- V-normalized power, thrust
      PC = PTOT * 2.0/PI
      TC = TTOT * 2.0/PI
C
C---- rpm-normalized power, thrust
      CP = PTOT * 0.25*(PI*ADV)**3
      CT = TTOT * 0.25*(PI*ADV)**2
C
C---- blade solidity
      CALL SPLINE(CH,W1,XI,II)
      CH34 = SEVAL(0.75,CH,W1,XI,II)
      SIGMA = FLOAT(NBLDS)*CH34/PI
C
C---- blade angles in degrees
C---- twist defined from hub station to tip
      BHUB = BETA(1)  * RTD
      BTIP = BETA(II) * RTD
      BTWIST = BHUB - BTIP
C---- twist defined from axis (extrapolated) to tip
c      B2   = BETA(2)  * RTD
c      BEXT = BHUB - XI(1)*(BHUB-B2)/(XI(1)-XI(2))
c      BTWIST = BEXT - BTIP
C
      RPM = VEL/(RAD*ADV) * 30.0/PI
C
C---- define sigma and helicopter related data
      IF(ADV.LT.0.1) THEN
       CTH  = CT/7.7516
       CPH  = CP/24.352
       CTOS = CTH / SIGMA
       FOM = 0.7979 * ABS(CT)**1.5 / CP
       LHELI = .TRUE.
      ENDIF
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0, 1.0,0.2    , 0.0,0.2, CS,1)
      CALL NEWPEN(3)
      CALL PLCHAR(0.7-1.5*CSL,-3.0*CSL,CSL,'r/R',0.0,3)
C
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = 5
       CALL PLGRID(0.0,0.0, NXG,0.2, NYG,0.2*PLPAR, LMASK2 )
      ENDIF
C
C---- set label locations
      YL = PLPAR + 14.0*CSL
      XL1 = 0.0
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
C
      CALL NEWPEN(4)
      CALL PLCHAR(XL1,YL,1.2*CSL,NAME,0.0,31)
C
      YL = YL - 1.0*CS
      CALL NEWPEN(1)
      CALL PLOT(0.0,YL,3)
      CALL PLOT(1.0,YL,2)
C
      CALL NEWPEN(3)
      YL = YL - 2.5*CS
      CALL PLCHAR(XL1 ,YL,CS,'#bld= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, FLOAT(NBLDS),0.0,-1)
      CALL PLCHAR(XL2 ,YL,CS,'R m = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RAD     ,0.0,3)
      CALL PLSUBS(XL3 ,YL,0.8*CS,'3/4',0.0,3,PLCHAR)
      CALL PLMATH(XL3 ,YL,CS,'s   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, SIGMA   ,0.0,4)
      CALL PLSUBS(XL4 ,YL,CS,'twist'  ,0.0,5,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTWIST  ,0.0,3)
C
      YL = YL - 2.5*CS
      CALL PLCHAR(XL1 ,YL,CS,'Vm/s= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, VEL     ,0.0,3)
      CALL PLCHAR(XL2, YL,CS,'   R  ' ,0.0,6)
      CALL PLMATH(XL2, YL,CS,'  W   ' ,0.0,6)
      CALL PLCHAR(XL2, YL,CS,' /    ' ,0.0,6)
      CALL PLCHAR(XL2, YL,CS,'V   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, ADV    ,0.0,4)
      CALL PLSUBS(XL3 ,YL,CS,'C'      ,0.0,1,PLCHAR)
      CALL PLCHAR(XL3 ,YL,CS,'P   = ' ,0.0,6)
      IF(ADV.GT.0.01) CALL PLNUMB(999.,YL,CS,PC,0.0,3)
      CALL PLSUBS(XL4 ,YL,CS,'P'      ,0.0,1,PLCHAR)
      CALL PLCHAR(XL4 ,YL,CS,'C   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, CP      ,0.0,4)
C---- induced efficiency (including nacelle thrust effect)
      EFFIND = TWAK/PWAK
C---- ideal (actuator disk) efficiency
      TCLIM = MAX( -1.0 , TC )
      EIDEAL = 2.0 / (1.0 + SQRT(TCLIM + 1.0))
      IF(WIND) THEN
       CALL PLCHAR(XL5-2.0*CS ,YL,CS,'1/' ,0.0,2)
       CALL PLSUBS(XL5 ,YL,0.9*CS,'ideal'  ,0.0,5,PLCHAR)
       CALL PLMATH(XL5 ,YL,CS,'h   = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS,1.0/EFFIND,0.0,4)
      ELSE
       CALL PLSUBS(XL5 ,YL,0.9*CS,'ideal'  ,0.0,5,PLCHAR)
       CALL PLMATH(XL5 ,YL,CS,'h   = ' ,0.0,6)
C--- Bug (?) plot says Nideal which should NOT be confused with Ninduced - HHY
ccc    CALL PLNUMB(999.,YL,CSL,EFFIND,0.0,4)
       CALL PLNUMB(999.,YL,CS,EIDEAL,0.0,4)
      ENDIF
C
      YL = YL - 2.5*CS
c      CALL PLCHAR(XL1 ,YL,CS,'hKft= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS,3.281*ALT,0.0,0)
      CALL PLCHAR(XL1 ,YL,CS,'h km= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS,ALT,0.0,3)
      CALL PLCHAR(XL2 ,YL,CS,'J   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PI*ADV  ,0.0,4)
      CALL PLSUBS(XL3 ,YL,CS,'C'      ,0.0,1,PLCHAR)
      CALL PLCHAR(XL3 ,YL,CS,'T   = ' ,0.0,6)
      IF(ADV.GT.0.01) CALL PLNUMB(999.,YL,CS,TC,0.0,3)
      CALL PLSUBS(XL4 ,YL,CS,'T'      ,0.0,1,PLCHAR)
      CALL PLCHAR(XL4 ,YL,CS,'C   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, CT      ,0.0,4)
      IF(WIND) THEN
       CALL PLCHAR(XL5-2.0*CS ,YL,CS,'1/' ,0.0,2)
       CALL PLMATH(XL5 ,YL,CS,'h   = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS,PTOT/TTOT,0.0,4)
      ELSE
       CALL PLMATH(XL5 ,YL,CS,'h   = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS,TTOT/PTOT,0.0,4)
      ENDIF
C
      YL = YL - 2.5*CS
c      CALL PLCHAR(XL1 ,YL,CS,'T lb= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/4.45,0.0,1)
      CALL PLCHAR(XL1 ,YL,CS,'T kN= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, TDIM/1000.,0.0,4)
c      CALL PLCHAR(XL2 ,YL,CS,'P hp= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/746.,0.0,1)
      CALL PLCHAR(XL2 ,YL,CS,'P kW= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PDIM/1000.,0.0,4)
      CALL PLCHAR(XL3 ,YL,CS,'RPM = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RPM     ,0.0,1)
      CALL PLSUBS(XL4 ,YL,CS,'tip'    ,0.0,3,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTIP    ,0.0,3)
C
      YL = YL - 1.0*CS
      CALL NEWPEN(1)
      CALL PLOT(0.0,YL,3)
      CALL PLOT(1.0,YL,2)
      CALL NEWPEN(3)
C
      IF(LHELI) THEN
       YL = YL - 2.5*CS
       CALL PLCHAR(XL1 ,YL,CS,'Helicopter' ,0.0,-1)
       CALL PLSUBS(XL2 ,YL,CS,'TH'     ,0.0,2,PLCHAR)
       CALL PLCHAR(XL2 ,YL,CS,'C   = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS, CTH     ,0.0,6)
       CALL PLSUBS(XL3 ,YL,CS,'PH'     ,0.0,2,PLCHAR)
       CALL PLCHAR(XL3 ,YL,CS,'C   = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS, CPH     ,0.0,6)
       CALL PLSUBS(XL4 ,YL,CS,'TH'     ,0.0,2,PLCHAR)
       CALL PLCHAR(XL4 ,YL,CS,'C /'    ,0.0,3)
       CALL PLMATH(999.,YL,CS,'s= '    ,0.0,3)
       CALL PLNUMB(999.,YL,CS, CTOS    ,0.0,4)
       CALL PLCHAR(XL5 ,YL,CS,'FOM = ' ,0.0,6)
       CALL PLNUMB(999.,YL,CS, FOM     ,0.0,4)
C
c       YL = YL - 1.0*CS
c       CALL NEWPEN(1)
c       CALL PLOT(0.0,YL,3)
c       CALL PLOT(1.0,YL,2)
c       CALL NEWPEN(3)
      ENDIF
C
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,0.0, PLPAR,0.2*PLPAR, 0.0,0.2, CS,1)
C
C---- CL vs r/R
      CALL NEWCOLORNAME('red')
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,CL,0.0,1.0,0.0,-PLPAR/CLMAX1,1)
      ELSE
       CALL XYLINE(II,XI,CL,0.0,1.0,0.0, PLPAR/CLMAX1,1)
      ENDIF
C     
      IL = 2*(II/3)
      CALL NEWPEN(2)
      CALL YAXIS(1.0,0.0, PLPAR,0.2*PLPAR, 0.0,DCL,-CS,1)
      CALL NEWPEN(3)
      XPLT =                 XI(IL)
      IF(WIND) THEN
       YPLT = 1.0*CSL - PLPAR*CL(IL)/CLMAX1
       CALL PLCHAR(XPLT-1.2*CSL,YPLT,1.2*CSL,'-c',0.0,2)
      ELSE
       YPLT = 1.0*CSL + PLPAR*CL(IL)/CLMAX1
       CALL PLCHAR(XPLT,YPLT,1.2*CSL,'c',0.0,1)
      ENDIF
      CALL PLSUBS(XPLT,YPLT,1.2*CSL,'V',0.0,1,PLMATH)

C
C---- normalized Gamma vs r/R
      CALL NEWCOLORNAME('cyan')
ccc      CALL SCALIT(II,GAM,0.0,GFAC)
      EFFINV = PWAK/TWAK
      ZETA = 2.0*(EFFINV-1.0)
      GFAC = 0.5*FLOAT(NBLDS)/(PI*ADV*ZETA*EFFINV)
c
      CALL NEWPEN(4)
      CALL XYLINE(II,XI,GAM,0.0,1.0,0.0,GFAC*PLPAR,2)
C
      IL = II/5
cc      CALL NEWPEN(2)
cc      CALL YAXIS(0.0,0.0, PLPAR,0.2*PLPAR, 0.0,0.2, CS,1)
      CALL NEWPEN(3)
      XPLT = -10.0*CSL +             XI(IL)
      YPLT =   0.5*CSL + GFAC*PLPAR*GAM(IL)
      CALL PLCHAR(XPLT,YPLT,CSL,'B /2   VR',0.0,9)
      CALL PLMATH(XPLT,YPLT,CSL,' G  pl   ',0.0,9)
      CALL PLSUBS(XPLT+5.0*CSL,YPLT,CSL,'w',0.0,1,PLCHAR)
C
C---- local efficiency vs r/R
      CALL NEWCOLORNAME('green')
      DO I=1, II
C------ use equivalent prop to define efficiency
        VW = VWAK(I)
        UTOTW = URDUCT
        CALL UVADD(XI(I),WA,WT)
C
        CW = XI(I)/ADV  - WT  -  VW
        SW = UTOTW      + WA  +  VW*XW(I)/ADW
C
        EFFI = (CW/SW) * ADV/XW(I)
        W1(I) = EFFI*EFFP(I)
C
        IF(WIND) W1(I) = 1.0/W1(I)
      ENDDO
C
      CALL NEWPEN(4)
      CALL XYLINE(II,XI,W1,0.0,1.0,0.0,PLPAR,3)
C
      IL = II/5
      CALL NEWPEN(3)
      XPLT =                 XI(IL)
      YPLT = 1.0*CSL + PLPAR*W1(IL)
      CALL PLMATH(XPLT,YPLT,1.1*CSL,'h'    ,0.0,1)
      CALL PLSUBS(XPLT,YPLT,    CSL,'local',0.0,5,PLCHAR)
      IF(WIND) 
     &CALL PLCHAR(XPLT-2.2*CSL,YPLT,1.1*CSL,'1/',0.0,2)
C
C
C---- Mach vs r/R
      CALL NEWCOLORNAME('yellow')
      DO I=1, II
C
C------ use real prop to define Mach number
            CALL CSCALC(I,UTOT,WA,WT,
     &                  VT,VT_ADW,
     &                  VA,VA_ADW,
     &                  VD,VD_ADW,
     &                  CI,CI_ADV,CI_VT,
     &                  SI,             SI_VA,
     &                  W,  W_ADV, W_VT, W_VA,
     &                  PHI,P_ADV, P_VT, P_VA)
C
        W1(I) = W * VEL/VSO
      ENDDO
C
      CALL NEWPEN(4)
      CALL XYLINE(II,XI,W1,0.0,1.0,0.0,PLPAR,4)
C
      IL = II/5
      CALL NEWPEN(3)
      XPLT =                 XI(IL)
      YPLT = 0.5*CSL + PLPAR*W1(IL)
      CALL PLCHAR(XPLT,YPLT,CSL,'M',0.0,1)
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = 1.0
      XYFAC(2) = PLPAR
C
      RETURN
      END ! CLPLT





      SUBROUTINE UVIPLT
      INCLUDE 'XROTOR.INC'
C--------------------------------------
C     Plots induced velocities vs r/R
C--------------------------------------
      EXTERNAL PLCHAR,PLMATH
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- plot aspect ratio
      PLPAR = PAR
C
      PLFAC = 0.85
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.4
C
      BLDS = FLOAT(NBLDS)
      XIB = 0.67
      XIA = 0.33
      XIBX = 1.0
      XIAX = 1.0
C
C=========================================================================
C  Note: 
C     The velocity at the blade lifting line is defined by 
C     VT = tangential velocity (this is 1/2 of the tangential velocity in
C          the slipstream immediately downstream of the blade)
C     VA = axial velocity due to the blade thrust (for a propeller this is
C          1/2 of the axial velocity downstream in the developed wake)
C     VD = axial velocity at prop plane due to duct
C=========================================================================
C
      DO I = 1, II
C---- induced velocity at the blade lifting line 
C
        VT = VIND(3,I)
        VA = VIND(1,I)
        VD = 0.0
C------ duct induced axial velocity
        UDUCT     = 0.0
        VADUCT_VA = 1.0
        IF(DUCT) THEN
          UDUCT = URDUCT-1.0
          VADUCT_VA = 2.0*URDUCT
        ENDIF
        VD = VA * (VADUCT_VA - 1.0)
C
        W1(I) = VA
        W2(I) = VT
        W3(I) = VD + VA
C
C---- Circumferentially averaged velocity derived from circulation
        VT = 0.5*BLDS*GAM(I) / (2.0*PI*XI(I))
        VA = 0.5*BLDS*GAM(I) / (2.0*PI*ADW  )
C------ duct induced axial velocity
        VD = VA * (VADUCT_VA - 1.0)
        W4(I) = VA
        W5(I) = VT
        W6(I) = VD + VA
C
ccc     write(*,*) i,xi(i),w1(i),w2(i),w3(i),w4(i)
C---- find radial stations closest to 1/2 and 3/4 radius for labels
        IF(ABS(XI(I)-XIA).LT.XIAX) THEN
          XIAX = ABS(XI(I)-XIA)
          IA = I
        ENDIF
        IF(ABS(XI(I)-XIB).LT.XIBX) THEN
          XIBX = ABS(XI(I)-XIB)
          IB = I
        ENDIF
C
C---- dimensional induced velocities
        W1(I) = W1(I)*VEL
        W2(I) = W2(I)*VEL
        W3(I) = W3(I)*VEL
        W4(I) = W4(I)*VEL
        W5(I) = W5(I)*VEL
        W6(I) = W6(I)*VEL

      ENDDO
C
      WMIN = 0.
      WMAX = 0.
      DO I = 3, II-II/5
        WMIN = MIN( WMIN, W1(I), W2(I), W3(I), W4(I), W5(I), W6(I) )
        WMAX = MAX( WMAX, W1(I), W2(I), W3(I), W4(I), W5(I), W6(I) )
      ENDDO
C
      CALL SCALIT(1,WMIN,0.0,WMNFAC)
      CALL SCALIT(1,WMAX,0.0,WMXFAC)
      WFAC = MIN( WMNFAC , WMXFAC )
      WDEL = 1.0 / (5.0*WFAC)
      IF(WMIN .LT. 0.0) WMIN = -WDEL * AINT( -WMIN/WDEL + 0.99 )
      IF(WMAX .GT. 0.0) WMAX =  WDEL * AINT(  WMAX/WDEL + 0.99 )
C
      CALL AXISADJ(WMIN,WMAX,WSPAN,WDEL,ntics)
      WFAC = PLPAR / (WMAX - WMIN)
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(1.25,1.0,-3)
C
      CALL GETCOLOR(ICOL0)
C
      CALL PLOT(0.0,-WFAC*WMIN,-3)
C---- case title
      CALL NEWPEN(4)
      XT = 0.0
      YT = WFAC*WMAX + 1.5*CSL + 2.5*CS
      CALL PLCHAR(XT,YT,CSL,NAME,0.0,-1)
C
C================================================================
C---- add some helpful case data
      RPM = VEL/(RAD*ADV) * 30.0/PI
C---- dimensional power, thrust
      PDIM = PTOT*RHO*VEL**3*RAD**2
      TDIM = TTOT*RHO*VEL**2*RAD**2
C---- blade angles in degrees
      BTIP = BETA(II) * RTD
C
      XL1 = XT
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
      YL = YT - 2.5*CS
C
c      CALL PLCHAR(XL1 ,YL,CS,'T lb= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/4.45,0.0,1)
      CALL PLCHAR(XL1 ,YL,CS,'T kN= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, TDIM/1000.,0.0,4)
c      CALL PLCHAR(XL2 ,YL,CS,'P hp= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/746.,0.0,1)
      CALL PLCHAR(XL2 ,YL,CS,'P kW= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PDIM/1000.,0.0,4)
      CALL PLCHAR(XL3 ,YL,CS,'RPM = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RPM     ,0.0,1)
      CALL PLSUBS(XL4 ,YL,CS,'tip'    ,0.0,3,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTIP    ,0.0,3)
C================================================================
C
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,WFAC*WMIN, 1.0,0.2    , 0.0,0.2, CS,1)
      CALL NEWPEN(3)
      CALL PLCHAR(0.7-1.5*CSL,WFAC*WMIN-3.0*CSL,CSL,'r/R',0.0,3)
C
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = INT( (WMAX-WMIN)/WDEL + 0.0001 )
       CALL PLGRID(0.0,WFAC*WMIN, NXG,0.2, NYG,WFAC*WDEL, LMASK2 )
      ENDIF
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,WFAC*WMIN,WFAC*(WMAX-WMIN),WFAC*WDEL,
     &            WMIN,WDEL, CS,-2)
      CALL NEWPEN(3)

      XL = -5.0*CSL
      YL = WFAC*(WMAX-1.5*WDEL) - 0.5*CSL
      CALL PLCHAR(XL,YL,CSL,'V'  ,0.0,1)
      CALL PLCHAR(XL+1.2*CSL,YL-0.0*CSL,0.8*CSL,'m/s'  ,0.0,3)
ccc      XL = -3.5*CSL
ccc      YL = WFAC*(WMAX-1.5*WDEL) - 0.5*CSL
ccc      CALL PLCHAR(XL,YL,CSL,'v/V'  ,0.0,3)

c      CALL PLSUBS(XL,YL+0.6*CSL,CSL,'axi',0.0,3,PLCHAR)
c      CALL PLOT(XL        ,YL,3)
c      CALL PLOT(XL+3.0*CSL,YL,2)
c      CALL PLCHAR(XL+1.0*CSL,YL-1.4*CSL,CSL,'V'    ,0.0,1)
C     
C---- Vaxial vs r/R
      CALL NEWCOLORNAME('cyan')
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,W1,0.0,1.0,0.0,-WFAC,1)
       CALL XYLINE(II,XI,W4,0.0,1.0,0.0,-WFAC,2)
      ELSE
       CALL XYLINE(II,XI,W1,0.0,1.0,0.0, WFAC,1)
       CALL XYLINE(II,XI,W4,0.0,1.0,0.0, WFAC,2)
      ENDIF
      CALL NEWPEN(3)
      XL = XI(IA) + 1.5*CS
      IF(WIND) THEN
        YL = -WFAC*W4(IA) + 1.5*CS
      ELSE
        YL =  WFAC*W4(IA) + 1.5*CS
      ENDIF
      CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
      CALL PLSUBS(XL,YL+0.6*CS,CS,'axi',0.0,3,PLCHAR)
C
C---- Vduct (axial) vs r/R
      IF(DUCT) THEN
        CALL NEWCOLORNAME('green')
        CALL NEWPEN(4)
        IF(WIND) THEN
          CALL XYLINE(II,XI,W3,0.0,1.0,0.0,-WFAC,1)
          CALL XYLINE(II,XI,W6,0.0,1.0,0.0,-WFAC,2)
         ELSE
          CALL XYLINE(II,XI,W3,0.0,1.0,0.0, WFAC,1)
          CALL XYLINE(II,XI,W6,0.0,1.0,0.0, WFAC,2)
        ENDIF
        CALL NEWPEN(3)
        XL = XI(IB) + 1.5*CS
        IF(WIND) THEN
          YL = -WFAC*W3(IB) + 1.5*CS
         ELSE
          YL =  WFAC*W3(IB) + 1.5*CS
        ENDIF
        CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
        CALL PLSUBS(XL,YL+0.6*CS,CS,'axi+duct',0.0,8,PLCHAR)
      ENDIF
cc      CALL PLOT(XL       ,YL,3)
cc      CALL PLOT(XL+3.0*CS,YL,2)
cc      CALL PLCHAR(XL+1.0*CS,YL-1.4*CS,CS,'V'    ,0.0,1)
C
C---- Vtangential vs r/R
      CALL NEWCOLORNAME('red')
c      CALL NEWPEN(3)
c      XL = -3.5*CSL
c      YL = WFAC*(WMAX-0.5*WDEL) + 0.1*CSL
c      CALL PLCHAR(XL,YL+0.6*CSL,CSL,'v'  ,0.0,1)
c      CALL PLSUBS(XL,YL+0.6*CSL,CSL,'tan',0.0,3,PLCHAR)
c      CALL PLOT(XL        ,YL,3)
c      CALL PLOT(XL+3.0*CSL,YL,2)
c      CALL PLCHAR(XL+1.0*CSL,YL-1.4*CSL,CSL,'V'    ,0.0,1)
C     
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,W2,0.0,1.0,0.0,-WFAC,1)
       CALL XYLINE(II,XI,W5,0.0,1.0,0.0,-WFAC,2)
      ELSE
       CALL XYLINE(II,XI,W2,0.0,1.0,0.0, WFAC,1)
       CALL XYLINE(II,XI,W5,0.0,1.0,0.0, WFAC,2)
      ENDIF
      CALL NEWPEN(3)
      XL = XI(IB) + 1.5*CS
      IF(WIND) THEN
        YL = -WFAC*W5(IB) + 1.5*CS
      ELSE
        YL =  WFAC*W5(IB) + 1.5*CS
      ENDIF
      CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
      CALL PLSUBS(XL,YL+0.6*CS,CS,'tan',0.0,3,PLCHAR)
cc      CALL PLOT(XL       ,YL,3)
cc      CALL PLOT(XL+3.0*CS,YL,2)
cc      CALL PLCHAR(XL+1.0*CS,YL-1.4*CS,CS,'V'    ,0.0,1)
C
      CALL NEWCOLOR(ICOL0)
C
      W8(1) = -0.1    + 1.0*CSL
      W8(2) =         - 1.0*CSL
      W9(1) =  0.5*CSL
      W9(2) =  0.5*CSL
C
      XL = 0.05 
      YL = WFAC*WMAX - 2.0*CSL
      CALL NEWPEN(3)
      CALL PLCHAR(XL,YL,0.8*CSL,'Induced velocity at blade',0.0,-1)
C
      XL = 0.1  + 0.5*CSL 
      YL = YL - 2.0*CSL
      CALL NEWPEN(4)
      CALL XYLINE(2,W8,W9,-XL,1.0,-YL,1.0,1)
      CALL NEWPEN(3)
      CALL PLCHAR(XL,YL,0.8*CSL,'Velocity induced',0.0,-1)
C
      YL = YL - 2.0*CSL
      CALL NEWPEN(4)
      CALL XYLINE(2,W8,W9,-XL,1.0,-YL,1.0,2)
      CALL NEWPEN(3)
      CALL PLCHAR(XL,YL,0.8*CSL,'Circum Avg',0.0,-1)
C
      CALL PLFLUSH
C
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = 1.0
      IF(WIND) THEN
       XYFAC(2) = -WFAC
      ELSE
       XYFAC(2) =  WFAC
      ENDIF     
C
      RETURN
      END ! UVIPLT





      SUBROUTINE TRIPLT
      INCLUDE 'XROTOR.INC'
C--------------------------------------
C     Plots velocity triangles vs r/R
C--------------------------------------
      EXTERNAL PLCHAR,PLMATH
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      DTR = ATAN(1.0)/45.0
      RTD = 1.0/DTR
C
C---- plot aspect ratio
      PLPAR = PAR
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.4
C
      DO I = 1, II
        VT = VIND(3,I)
        VA = VIND(1,I)
C
C------ include duct effect on freestream and induced axial velocity
        UDUCT     = 0.0
        VADUCT_VA = 1.0
        IF(DUCT) THEN
          UDUCT = URDUCT-1.0
          VADUCT_VA = 2.0*URDUCT
        ENDIF
        VD = VA * (VADUCT_VA - 1.0)
        UTOT = 1.0 + UDUCT + UBODY(I)
C
        CALL UVADD(XI(I),WA,WT)
C
        W1(I) = XI(I)/ADV
        W2(I) = UTOT
C
        W3(I) = XI(I)/ADV - WT
        W4(I) = UTOT      + WA
C
        W5(I) = XI(I)/ADV - WT  -  VT
        W6(I) = UTOT      + WA  +  VA  + VD
      ENDDO
C
C
      PLFAC = 0.8
C
      UMAX = 1.0/ADV
      CALL SCALIT(1,UMAX,0.0,UFAC)
      UDEL = 1.0 / (5.0*UFAC)
      UMAX = UDEL * AINT( UMAX/UDEL + 0.99 )
      UFAC = PLPAR/UMAX
C
      RFAC = 1.0/(1.0+UFAC*W6(II))
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,.FALSE.)
      CALL PLOTABS(1.25,1.5,-3)
C---- case title
      CALL NEWPEN(4)
      XT = 0.0
      YT = -5.5*CSL
      CALL PLCHAR(XT,YT,CSL,NAME,0.0,-1)
C
C================================================================
C---- add some helpful case data
      RPM = VEL/(RAD*ADV) * 30.0/PI
C---- dimensional power, thrust
      PDIM = PTOT*RHO*VEL**3*RAD**2
      TDIM = TTOT*RHO*VEL**2*RAD**2
C---- blade angles in degrees
      BTIP = BETA(II) * RTD
C
      XL1 = XT
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
      YL = YT - 2.5*CS
C
c      CALL PLCHAR(XL1 ,YL,CS,'T lb= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/4.45,0.0,1)
      CALL PLCHAR(XL1 ,YL,CS,'T kN= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, TDIM/1000.,0.0,4)
c      CALL PLCHAR(XL2 ,YL,CS,'P hp= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/746.,0.0,1)
      CALL PLCHAR(XL2 ,YL,CS,'P kW= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PDIM/1000.,0.0,4)
      CALL PLCHAR(XL3 ,YL,CS,'RPM = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RPM     ,0.0,1)
      CALL PLSUBS(XL4 ,YL,CS,'tip'    ,0.0,3,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTIP    ,0.0,3)
C================================================================
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,0.0,RFAC,0.1*RFAC,0.0,0.1,CS, 1)
C
      CALL NEWPEN(3)
      XL = -4.0*CSL
      YL = RFAC*0.75 - 0.5*CSL
      CALL PLCHAR(XL,YL,CSL,'r/R',0.0,3)
C
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0      ,0.0,UFAC*UMAX,UFAC*UDEL,0.0,UDEL, CS,-2)
      CALL YAXIS(UFAC*UMAX,0.0,UFAC*UMAX,UFAC*UDEL,0.0,UDEL,-CS,-2)
C
      CALL NEWPEN(3)
      XL = UFAC*(UMAX - 0.5*UDEL) - 2.5*CSL
      YL =                        - 3.0*CSL
      CALL PLCHAR(XL,YL,CSL,'v  /V',0.0,5)
      CALL PLSUBS(XL,YL,CSL,'tan'  ,0.0,3,PLCHAR)
C
      XL = UFAC* UMAX             + 1.0*CSL
      YL = UFAC*(UMAX - 0.5*UDEL) - 0.5*CSL
      CALL PLCHAR(XL,YL,CSL,'v  /V',0.0,5)
      CALL PLSUBS(XL,YL,CSL,'axi'  ,0.0,3,PLCHAR)
C
C
      call newcolorname('black')
      CALL NEWPEN(1)
      DO I = 1, II
        CALL PLOT(0.0       ,RFAC*XI(I)           ,3)
        CALL PLOT(UFAC*W1(I),RFAC*XI(I)           ,2)
        CALL PLOT(UFAC*W1(I),RFAC*XI(I)+UFAC*W2(I),2)
      ENDDO
C
      call newcolorname('cyan')
      CALL NEWPEN(3)
      DO I = 1, II
        CALL PLOT(UFAC*W1(I),RFAC*XI(I)+UFAC*W2(I),3)
        CALL PLOT(UFAC*W3(I),RFAC*XI(I)+UFAC*W4(I),2)
      ENDDO
      I = II
      XL =              UFAC*0.5*(W1(I)+W3(I)) + 0.75*CS
      YL = RFAC*XI(I) + UFAC*0.5*(W2(I)+W4(I)) + 0.25*CS
      CALL PLCHAR(XL,YL,CS,'Imposed',0.0,7)
C
      call newcolorname('red')
      CALL NEWPEN(3)
      DO I = 1, II
        CALL PLOT(UFAC*W5(I),RFAC*XI(I)+UFAC*W6(I),3)
        CALL PLOT(UFAC*W3(I),RFAC*XI(I)+UFAC*W4(I),2)
      ENDDO
      I = II
      XL =              UFAC*0.5*(W3(I)+W5(I)) + 0.75*CS
      YL = RFAC*XI(I) + UFAC*0.5*(W4(I)+W6(I)) + 0.25*CS
      CALL PLCHAR(XL,YL,CS,'Induced',0.0,7)
C
      call newcolorname('green')
      CALL NEWPEN(4)
      DO I = 1, II
        CALL PLOT(0.0       ,RFAC*XI(I)           ,3)
        CALL PLOT(UFAC*W5(I),RFAC*XI(I)+UFAC*W6(I),2)
      ENDDO
      I = II
      XL =              UFAC*0.5*W5(I) - 5.75*CS
      YL = RFAC*XI(I) + UFAC*0.5*W6(I) + 0.25*CS
      CALL PLCHAR(XL,YL,CS,'Total',0.0,5)
C
      call newcolorname('black')
      CALL PLFLUSH
C
C---- no x-y plot on screen currently
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = 0.
      XYFAC(2) = 0.
C
      RETURN
      END ! TRIPLT



      SUBROUTINE VELPLT
      INCLUDE 'XROTOR.INC'
C------------------------------------------------------
C     Plots imposed slipstream Vaxi and Vtan vs r/R
C------------------------------------------------------
      EXTERNAL PLCHAR,PLMATH
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      DTR = ATAN(1.0)/45.0
      RTD = 1.0/DTR
C
C---- plot scale factor and aspect ratio
      PLPAR = PAR
      PLFAC = 0.85
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.2
C
      DO I=1, NADD
        W1(I) = RADD(I)/RAD
        W2(I) = UADD(I)
        W3(I) = VADD(I)
      ENDDO
C
      YMIN = MIN(W2(1),W3(1))
      YMAX = MAX(W2(1),W3(1))
      DO I=2, NADD
        YMIN = MIN(W2(I),W3(I),YMIN)
        YMAX = MAX(W2(I),W3(I),YMAX)
      ENDDO
      IF(YMIN.EQ.YMAX) THEN
        YMAX = YMIN + 1.0
      ENDIF
      CALL AXISADJ(YMIN,YMAX,YSPAN,YDEL,ntics)
      YSF = PLPAR / (YMAX - YMIN)
C
      XDEL = 0.2
      XMIN = 0.0
      XMAX = 1.0
      XSF  = 1.0
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL GETCOLOR(ICOL0)
C
      CALL PLOTABS(1.0,0.5,-3)
      CALL PLOT(0.0,-YMIN*YSF,-3)
C
      CALL NEWPEN(2)
      CALL XAXIS(XMIN*XSF,0.0,-1.0 ,XDEL*XSF,XMIN,XDEL,CS, 1)
      CALL PLCHAR(0.7-1.5*CSL,-3.0*CSL,CSL,'r/R',0.0,3)
C
      CALL YAXIS(0.0,YMIN*YSF,PLPAR,YDEL*YSF,YMIN,YDEL,CS,-2)
      XL = -5.0*CSL
      YL = YSF*(YMAX-1.5*YDEL) - 0.5*CSL
      CALL PLCHAR(XL,YL,CSL,'V'  ,0.0,1)
      CALL PLCHAR(XL+1.2*CSL,YL-0.0*CSL,0.8*CSL,'m/s'  ,0.0,3)
C
C---- case title
      CALL NEWPEN(4)
      XT = 0.0
      YT = YSF*YMAX + 3.0*CSL + 2.5*CS
      CALL PLCHAR(XT,YT,CSL,NAME,0.0,-1)
C
      YT = YT - 1.5*CSL
      CALL PLCHAR(0.0,YT,CS,'Imposed propeller velocity',0.0,-1)
C
C================================================================
C---- add some helpful case data
      RPM = VEL/(RAD*ADV) * 30.0/PI
C---- dimensional power, thrust
      PDIM = PTOT*RHO*VEL**3*RAD**2
      TDIM = TTOT*RHO*VEL**2*RAD**2
C---- blade angles in degrees
      BTIP = BETA(II) * RTD
C
      XL1 = XT
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
      YL = YT - 2.5*CS
C
c      CALL PLCHAR(XL1 ,YL,CS,'T lb= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/4.45,0.0,1)
      CALL PLCHAR(XL1 ,YL,CS,'T kN= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, TDIM/1000.,0.0,4)
c      CALL PLCHAR(XL2 ,YL,CS,'P hp= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/746.,0.0,1)
      CALL PLCHAR(XL2 ,YL,CS,'P kW= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PDIM/1000.,0.0,4)
      CALL PLCHAR(XL3 ,YL,CS,'RPM = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RPM     ,0.0,1)
      CALL PLSUBS(XL4 ,YL,CS,'tip'    ,0.0,3,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTIP    ,0.0,3)
C================================================================
C
      IF(NADD.GT.0) THEN
       IL = NADD/4 + 1
       CALL NEWCOLORNAME('red')
       CALL NEWPEN(3)
       CALL XYLINE(NADD,W1,W2,0.0,XSF,0.0,YSF,2)
C
       CALL NEWPEN(2)
       XPLT = XSF*W1(IL)
       YPLT = YSF*W2(IL) + 1.5*CSL
       CALL PLCHAR(XPLT,YPLT,1.2*CSL,'v',0.0,1)
       CALL PLSUBS(XPLT+1.2*CSL,YPLT,1.2*CSL,'axi'  ,0.0,3,PLCHAR)
C
       IL = NADD/3 + 1
       CALL NEWCOLORNAME('orange')
       CALL NEWPEN(3)
       CALL XYLINE(NADD,W1,W3,0.0,XSF,0.0,YSF,3)
C
       CALL NEWPEN(2)
       XPLT = XSF*W1(IL)
       YPLT = YSF*W3(IL) + 1.5*CSL
       CALL PLCHAR(XPLT,YPLT,1.2*CSL,'v',0.0,1)
       CALL PLSUBS(XPLT+1.2*CSL,YPLT,1.2*CSL,'tan'  ,0.0,3,PLCHAR)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT( (XMAX-XMIN)/(0.5*XDEL) + 0.01 )
       NYG = INT( (YMAX-YMIN)/(0.5*YDEL) + 0.01 )
       CALL PLGRID(XSF*XMIN,YSF*YMIN, 
     &             NXG,XSF*0.5*XDEL, NYG,YSF*0.5*YDEL, LMASK2)
      ENDIF
C
      CALL PLFLUSH
C
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = XSF
      XYFAC(2) = YSF
C
      RETURN
      END ! VELPLT



      SUBROUTINE NACPLT(NB,ZB,RB,ZPROP,RW, N,R,U)
      DIMENSION ZB(*), RB(*), R(*), U(*)
C--------------------------------------------------
C     Plots nacelle, and velocity profile at disk.
C--------------------------------------------------
      ZMOD(ZZ) = (ZZ - ZOFF)*ZSF
      RMOD(RR) =  RR        *ZSF
C
      SH = 0.003
C
      CALL GETCOLOR(ICOL0)
C
      ZSF = 0.90 / (ZB(NB) - ZB(1))
      RSF = 0.70 / R(N)
C
      ZSF  = MIN( ZSF , RSF )
      ZOFF = ZB(1) - 0.025/ZSF
C
      USF = 0.5 / R(N)
C
C
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
C
      CALL NEWCOLORNAME('green')
      CALL NEWPEN(3)
      IB = 1
      CALL PLSYMB(ZMOD(ZB(IB)),RMOD(RB(IB)),SH,1,0.0,0)
      DO IB=2, NB
        CALL PLSYMB(ZMOD(ZB(IB)),RMOD(RB(IB)),SH,1,0.0,1)
      ENDDO
C
C--- Plot body extension
      CALL NEWPAT(-30584)
      CALL PLOT(ZMOD(ZB(NB))     ,RMOD(RW),3)
      CALL PLOT(ZMOD(ZB(NB))+0.05,RMOD(RW),2)
      CALL NEWPAT(-1)
C
C--- Plot prop
      CALL NEWCOLORNAME('red')
      CALL NEWPEN(5)
      CALL PLOT(ZMOD(ZPROP),RMOD(R(1)),3)
      CALL PLOT(ZMOD(ZPROP),RMOD(R(N)),2)
C
C--- Plot axis for U velocity
      CALL NEWCOLORNAME('black')
      CALL NEWPEN(2)
      CALL NEWPAT(-21846)
      CALL PLOT(ZMOD(ZPROP+1.0*USF),RMOD(R(1)),3)
      CALL PLOT(ZMOD(ZPROP+1.0*USF),RMOD(R(N)),2)
      CALL NEWPAT(-1)
C
C--- Plot U velocity over prop radius range
      CALL NEWCOLORNAME('orange')
      CALL NEWPEN(2)
      I = 1
      CALL PLOT(ZMOD(ZPROP+(1.0+U(I))*USF),RMOD(R(I)),3)
      DO I=2, N
        CALL PLOT(ZMOD(ZPROP+(1.0+U(I))*USF),RMOD(R(I)),2)
      ENDDO
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
      RETURN
      END ! NACPLT




      SUBROUTINE REFPLT(FNAME, XOFF,YOFF,XFAC,YFAC, SSIZE,ISYM)
      CHARACTER*(*) FNAME
C
      PARAMETER (NDIM=200)
      DIMENSION X(NDIM), Y(NDIM)
      CHARACTER*128 LINE
C
      LU = 49
C
      IF(FNAME(1:1) .EQ. ' ') THEN
       CALL ASKS('Enter x,y data filename^',FNAME)
      ENDIF
C
 1000 FORMAT(A)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=90)
      XSCALE = 1.0
      YSCALE = 1.0
C
      DO 10 I=1, NDIM
        READ(LU,1000,END=11) LINE
C
        K = INDEX(LINE,'*') + 1
        IF(K.EQ.2) THEN
          READ(LINE(K:128),*,ERR=11) XSCALE, YSCALE
          GO TO 10
        ENDIF
C
        K = INDEX(LINE,'#')
        IF(K.EQ.1) GO TO 10
C
        READ(LINE,*,ERR=11) X(I), Y(I)
        X(I) = X(I) * XSCALE
        Y(I) = Y(I) * YSCALE
C
 10   CONTINUE
 11   CONTINUE
C
      N = I-1
      CLOSE(LU)
C
      CALL XYSYMB(N,X,Y,XOFF,XFAC,YOFF,YFAC,SSIZE,ISYM)
C
      CALL PLFLUSH
      RETURN
C
 90   CONTINUE
      WRITE(*,*) 'REFPLT: File not found'
      RETURN
C
      END



      SUBROUTINE PRPPLT
      INCLUDE 'XROTOR.INC'
C------------------------------------------
C     Plots prop outline (all blades)
C------------------------------------------
      EXTERNAL PLCHAR,PLMATH
      DIMENSION XLIN(4), YLIN(4)
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      DTR = ATAN(1.0)/45.0
      RTD = 1.0/DTR
C
C---- disk area
      ADISK = PI*RAD**2 * (1.0 - XI0**2)
C
C---- blade solidity
      CALL SPLINE(CH,W1,XI,II)
      CH34 = SEVAL(0.75,CH,W1,XI,II)
      SIGMA = FLOAT(NBLDS)*CH34/PI
C
C---- blade angles in degrees
C---- twist defined from hub station to tip
      BHUB = BETA(1)  * RTD
      BTIP = BETA(II) * RTD
      BTWIST = BHUB - BTIP
C---- twist defined from axis (extrapolated) to tip
c      B2   = BETA(2)  * RTD
c      BEXT = BHUB - XI(1)*(BHUB-B2)/(XI(1)-XI(2))
c      BTWIST = BEXT - BTIP
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.4
C
C---- case title
      CALL NEWPEN(4)
      XL1 = 0.0
      YL = 5.0*CS
      CALL PLCHAR(XL1,YL,CSL,NAME,0.0,-1)
C
      CALL NEWPEN(3)
      XL1 = 0.0
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
      YL = YL - 2.5*CS
      CALL PLCHAR(XL1 ,YL,CS,'#bld = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, FLOAT(NBLDS),0.0,-1)
      CALL PLCHAR(XL2 ,YL,CS,'R m  = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, RAD      ,0.0,4)
      CALL PLSUPS(XL3+2.0*CS ,YL,0.8*CS,'2' ,0.0,1,PLCHAR)
      CALL PLCHAR(XL3 ,YL,CS,'A m  = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, ADISK    ,0.0,6)
C
      YL = YL - 2.5*CS
      CALL PLSUBS(XL1 ,YL,0.8*CS,'3/4' ,0.0,3,PLCHAR)
      CALL PLMATH(XL1 ,YL,CS,'s    = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, SIGMA    ,0.0,4)
      CALL PLSUBS(XL2 ,YL,0.8*CS,'hub' ,0.0,3,PLCHAR)
      CALL PLCHAR(XL2 ,YL,CS,'R    = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, XI0*RAD  ,0.0,4)
C
      YL = YL - 2.5*CS
      CALL PLSUBS(XL1 ,YL,CS,'twist'   ,0.0,5,PLCHAR)
      CALL PLMATH(XL1 ,YL,CS,'b    = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, BTWIST   ,0.0,3)
      CALL PLSUBS(XL2 ,YL,0.8*CS,'wak' ,0.0,3,PLCHAR)
      CALL PLCHAR(XL2 ,YL,CS,'R    = ' ,0.0,7)
      CALL PLNUMB(999.,YL,CS, XW0*RAD  ,0.0,4)
C
C---- vertical space/R between projections
C
      CHUMAX = 0.
      CHLMAX = 0.
      DO I=1, II
        SINB = SIN(BETA(I))
        COSB = COS(BETA(I))
C
C------ projection chords in front and behind radial axis
        W1(I) =  XPITCH     *SINB*CH(I)
        W2(I) = (XPITCH-1.0)*SINB*CH(I)
C
        W3(I) =  XPITCH     *COSB*CH(I)
        W4(I) = (XPITCH-1.0)*COSB*CH(I)
C
        W5(I) =  XPITCH     *CH(I)
        W6(I) = (XPITCH-1.0)*CH(I)
C
        CHUMAX = MAX(CHUMAX,     CH(I))
        CHLMAX = MAX(CHLMAX,SINB*CH(I))
      END DO
C
C
C---- plot axial view 
        CALL PLOT(0.5,1.2*PAR,-3)
        CALL NEWPEN(1)
        CALL GETFACTORS(XSCALE,YSCALE)
        CALL NEWFACTOR(0.5*XSCALE)
C
C------ plot radial axis
        CALL PLOT(0.0,0.0,3)
        CALL PLOT(1.0,0.0,2)
C
C------ for axial view, plot blade "spokes" (A)
        RSPOKE = MAX( 0.1 , 1.25*XI0 )
        DO IB=2, NBLDS
          ANG = 2.0*PI * FLOAT(IB-1)/FLOAT(NBLDS)
          XP = RSPOKE*COS(ANG)
          YP = RSPOKE*SIN(ANG)
          CALL PLOT(0.0,0.0,3)
          CALL PLOT(XP,YP,2)
        ENDDO
C
C------ plot radial tick marks on #1 (horizontal) blade
        DO NT=0, 5
          XT = 0.2*FLOAT(NT)
          CALL PLOT(XT,-.005,3)
          CALL PLOT(XT,0.005,2)
        ENDDO
C
C------ plot tip circle for duct
        IF(DUCT) THEN
          CALL NEWPEN(2)
          CALL PLCIRC(0.0,0.0,1.0,72)
        ENDIF
C
C------ plot hub circle
        CALL NEWPEN(2)
        CALL PLCIRC(0.0,0.0,XI0,0)
C
C------ plot wake circle
        CALL NEWPEN(2)
        CALL PLCIRC(0.0,0.0,XW0,18)
C
C------ plot blade shape
        CALL GETCOLOR(ICOL0)
        CALL NEWCOLORNAME('yellow')
        CALL NEWPEN(3)
        DO IB=1, NBLDS
          ANG = 2.0*PI * FLOAT(IB-1)/FLOAT(NBLDS)
          SINA = SIN(ANG)
          COSA = COS(ANG)
          DO I = 1, II
c--- chordline defined on circular cylinder at radius XI(I)
             THET = W3(I)/XI(I)
             XX = XI(I)*COS(THET)
             YY = XI(I)*SIN(THET)
             XP = XX*COSA - YY*SINA
             YP = XX*SINA + YY*COSA
c--- chordline defined on plane normal to stacking line
c            XP = XI(I)*COSA - W3(I)*SINA
c            YP = XI(I)*SINA + W3(I)*COSA
            IF(I.EQ.1) THEN
              CALL PLOT(XP,YP,3)
             ELSE
              CALL PLOT(XP,YP,2)
            ENDIF
          ENDDO
          DO I = 1, II
c--- chordline defined on circular cylinder at radius XI(I)
             THET = W4(I)/XI(I)
             XX = XI(I)*COS(THET)
             YY = XI(I)*SIN(THET)
             XP = XX*COSA - YY*SINA
             YP = XX*SINA + YY*COSA
c--- chordline defined on plane normal to stacking line
c            XP = XI(I)*COSA - W4(I)*SINA
c            YP = XI(I)*SINA + W4(I)*COSA
            IF(I.EQ.1) THEN
              CALL PLOT(XP,YP,3)
             ELSE
              CALL PLOT(XP,YP,2)
            ENDIF
          ENDDO
        ENDDO
cc        CALL XYLINE(II,XI,W3,0.0,1.0,0.0,1.0,1)
cc        CALL XYLINE(II,XI,W4,0.0,1.0,0.0,1.0,1)
        CALL NEWCOLOR(ICOL0)
C
C
C
      CALL PLFLUSH
      RETURN
      END ! PRPPLT




      SUBROUTINE UVIPLT3
      INCLUDE 'XROTOR.INC'
C--------------------------------------------------------------------
C     Plots velocities in slipstream immediately downstream of the
C     rotor (vs r/R)
C     
C     Note: the near-rotor slipstream velocities
C     are calculated just downstream of the rotor
C     plane. 
C     Axial induced vel. derives from circulation and wake advance ratio
C     Total swirl vel. derives from differential torque
C--------------------------------------------------------------------
      EXTERNAL PLCHAR,PLMATH
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      DTR = ATAN(1.0)/45.0
      RTD = 1.0/DTR
C
C---- plot aspect ratio
      PLPAR = PAR
      PLFAC = 0.85
C
C---- character size for axis numbers, labels
      CS  = CSIZE
      CSL = CSIZE*1.4
C
      BLDS = FLOAT(NBLDS)
      OMEG = VEL/(ADV*RAD)
C
      XIB = 0.67
      XIA = 0.33
      XIBX = 1.0
      XIAX = 1.0
C
C=========================================================================
C  Note: 
C     The velocity at the blade lifting line is defined by 
C     VT = tangential velocity (this is 1/2 of the tangential velocity in
C          the slipstream immediately downstream of the blade)
C     VA = axial velocity due to the blade thrust (for a propeller this is
C          1/2 of the axial velocity downstream in the developed wake)
C     VD = axial velocity at prop plane due to duct
C=========================================================================
C
      DO I = 1, II

C------ use circumferentially averaged induced velocities 
        VT = BLDS*GAM(I)/(4.0*PI*XI(I))
        VA = VT*XI(I)/ADW
C------ include duct effect on freestream and induced axial velocity
        UDUCT     = 0.0
        VADUCT_VA = 1.0
        IF(DUCT) THEN
          UDUCT = URDUCT-1.0
          VADUCT_VA = 2.0*URDUCT
        ENDIF
C
C------ duct induced axial velocity
        VD = VA * (VADUCT_VA - 1.0)
C
C------ total axial velocity downstream of rotor
        UTOT = 1.0 + UDUCT + UBODY(I)
        CALL UVADD(XI(I),WA,WT)
        CI = XI(I)/ADV - WT  -  VT
        SI = UTOT      + WA  +  VA + VD
C
C------ derive tangential velocity from torque (includes blade drag torque)
        DPP = (DPII(I) + DPVI(I)) * (RHO * VEL**3 * RAD**2)
        DQQ = DPP/OMEG
        DQQ  = DQQ / (RHO * VEL**2 * RAD**3)
        VTA = BLDS*DQQ / (2.0*PI*(1.0+VA+VD)*(XI(I)**2))
        ASLIP = ATAN2(VTA,SI) * RTD
C
C------ Plot axial induced velocity, total swirl, duct+induced axial velocity
C       and total axial velocity (includes free stream).
        W1(I) = VA
        W2(I) = VTA
        W3(I) = VD + VA
        W4(I) = SI
C
C
ccc     write(*,*) i,xi(i),w1(i),w2(i),w3(i),w4(i)
C---- find radial stations closest to 1/2 and 3/4 radius for labels
        IF(ABS(XI(I)-XIA).LT.XIAX) THEN
          XIAX = ABS(XI(I)-XIA)
          IA = I
        ENDIF
        IF(ABS(XI(I)-XIB).LT.XIBX) THEN
          XIBX = ABS(XI(I)-XIB)
          IB = I
        ENDIF

C---- dimensional induced velocities
        W1(I) = W1(I)*VEL
        W2(I) = W2(I)*VEL
        W3(I) = W3(I)*VEL
        W4(I) = W4(I)*VEL

      ENDDO
C
      WMIN = 0.
      WMAX = 0.
      DO I = 3, II-II/5
        WMIN = MIN( WMIN, W1(I), W2(I), W3(I), W4(I) )
        WMAX = MAX( WMAX, W1(I), W2(I), W3(I), W4(I) )
      ENDDO
C
      CALL SCALIT(1,WMIN,0.0,WMNFAC)
      CALL SCALIT(1,WMAX,0.0,WMXFAC)
      WFAC = MIN( WMNFAC , WMXFAC )
      WDEL = 1.0 / (5.0*WFAC)
      IF(WMIN .LT. 0.0) WMIN = -WDEL * AINT( -WMIN/WDEL + 0.99 )
      IF(WMAX .GT. 0.0) WMAX =  WDEL * AINT(  WMAX/WDEL + 0.99 )
C
      CALL AXISADJ(WMIN,WMAX,WSPAN,WDEL,ntics)
      WFAC = PLPAR / (WMAX - WMIN)
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(1.25,1.0,-3)
C
      CALL GETCOLOR(ICOL0)
C
      CALL PLOT(0.0,-WFAC*WMIN,-3)
C---- case title
      CALL NEWPEN(4)
      XT = 0.0
      YT = WFAC*WMAX + 1.5*CSL + 2.5*CS
      CALL PLCHAR(XT,YT,CSL,NAME,0.0,-1)
C
C================================================================
C---- add some helpful case data
      RPM = VEL/(RAD*ADV) * 30.0/PI
C---- dimensional power, thrust
      PDIM = PTOT*RHO*VEL**3*RAD**2
      TDIM = TTOT*RHO*VEL**2*RAD**2
C---- blade angles in degrees
      BTIP = BETA(II) * RTD
C
      XL1 = XT
      XL2 = XL1 + 17.0*CS
      XL3 = XL2 + 17.0*CS
      XL4 = XL3 + 17.0*CS
      XL5 = XL4 + 17.0*CS
      YL = YT - 2.5*CS
C
c      CALL PLCHAR(XL1 ,YL,CS,'T lb= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/4.45,0.0,1)
      CALL PLCHAR(XL1 ,YL,CS,'T kN= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, TDIM/1000.,0.0,4)
c      CALL PLCHAR(XL2 ,YL,CS,'P hp= ' ,0.0,6)
c      CALL PLNUMB(999.,YL,CS, TDIM/746.,0.0,1)
      CALL PLCHAR(XL2 ,YL,CS,'P kW= ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, PDIM/1000.,0.0,4)
      CALL PLCHAR(XL3 ,YL,CS,'RPM = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, RPM     ,0.0,1)
      CALL PLSUBS(XL4 ,YL,CS,'tip'    ,0.0,3,PLCHAR)
      CALL PLMATH(XL4 ,YL,CS,'b   = ' ,0.0,6)
      CALL PLNUMB(999.,YL,CS, BTIP    ,0.0,3)
C================================================================
C
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
c      CALL PLOT(1.0,0.0,2)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,WFAC*WMIN, 1.0,0.2    , 0.0,0.2, CS,1)
      CALL NEWPEN(3)
      CALL PLCHAR(0.7-1.5*CSL,WFAC*WMIN-3.0*CSL,CSL,'r/R',0.0,3)
C
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 5
       NYG = INT( (WMAX-WMIN)/WDEL + 0.0001 )
       CALL PLGRID(0.0,WFAC*WMIN, NXG,0.2, NYG,WFAC*WDEL, LMASK2 )
      ENDIF
C
      CALL NEWPEN(2)
      CALL YAXIS(0.0,WFAC*WMIN,WFAC*(WMAX-WMIN),WFAC*WDEL,
     &            WMIN,WDEL, CS,-2)
      CALL NEWPEN(3)

      XL = -5.0*CSL
      YL = WFAC*(WMAX-1.5*WDEL) - 0.5*CSL
      CALL PLCHAR(XL,YL,CSL,'V',0.0,1)
      CALL PLCHAR(XL+1.2*CSL,YL-0.0*CSL,0.8*CSL,'m/s',0.0,3)
C     
C---- VArotor vs r/R
      CALL NEWCOLORNAME('cyan')
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,W1,0.0,1.0,0.0,-WFAC,1)
      ELSE
       CALL XYLINE(II,XI,W1,0.0,1.0,0.0, WFAC,1)
      ENDIF
      CALL NEWPEN(3)
      XL = XI(IA) + 1.5*CS
      IF(WIND) THEN
        YL = -WFAC*W1(IA) + 1.5*CS
      ELSE
        YL =  WFAC*W1(IA) + 1.5*CS
      ENDIF
      CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
      CALL PLSUBS(XL,YL+0.6*CS,CS,'axi',0.0,3,PLCHAR)
C
C---- VTslipstream vs r/R
      CALL NEWCOLORNAME('red')
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,W2,0.0,1.0,0.0,-WFAC,2)
      ELSE
       CALL XYLINE(II,XI,W2,0.0,1.0,0.0, WFAC,2)
      ENDIF
      CALL NEWPEN(3)
      XL = XI(IB) + 1.5*CS
      IF(WIND) THEN
        YL = -WFAC*W2(IB) + 1.5*CS
      ELSE
        YL =  WFAC*W2(IB) + 1.5*CS
      ENDIF
      CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
      CALL PLSUBS(XL,YL+0.6*CS,CS,'tan',0.0,3,PLCHAR)
C
C---- Vduct + VArotor vs r/R
      IF(DUCT) THEN
        CALL NEWCOLORNAME('green')
        CALL NEWPEN(4)
        IF(WIND) THEN
          CALL XYLINE(II,XI,W3,0.0,1.0,0.0,-WFAC,3)
         ELSE
          CALL XYLINE(II,XI,W3,0.0,1.0,0.0, WFAC,3)
        ENDIF
        CALL NEWPEN(3)
        XL = XI(IB) + 1.5*CS
        IF(WIND) THEN
          YL = -WFAC*W3(IB) + 1.5*CS
         ELSE
          YL =  WFAC*W3(IB) + 1.5*CS
        ENDIF
        CALL PLCHAR(XL,YL+0.6*CS,CS,'v'  ,0.0,1)
        CALL PLSUBS(XL,YL+0.6*CS,CS,'axi+duct',0.0,8,PLCHAR)
      ENDIF
C     
C---- VAtotal vs r/R
      CALL NEWCOLORNAME('magenta')
      CALL NEWPEN(4)
      IF(WIND) THEN
       CALL XYLINE(II,XI,W4,0.0,1.0,0.0,-WFAC,4)
      ELSE
       CALL XYLINE(II,XI,W4,0.0,1.0,0.0, WFAC,4)
      ENDIF
      CALL NEWPEN(3)
      XL = XI(IA) + 1.5*CS
      IF(WIND) THEN
        YL = -WFAC*W4(IA) + 1.5*CS
      ELSE
        YL =  WFAC*W4(IA) + 1.5*CS
      ENDIF
      CALL PLCHAR(XL,YL+0.6*CS,CS,'V+v'  ,0.0,3)
      IF(DUCT) THEN
        CALL PLSUBS(XL+2.*CS,YL+0.6*CS,CS,'axi+duct',0.0,8,PLCHAR)
       ELSE
        CALL PLSUBS(XL+2.*CS,YL+0.6*CS,CS,'axi',0.0,3,PLCHAR)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
      XL = 0.0 + 2.0*CSL
      YL = WFAC*WMAX - 2.0*CSL
      CALL NEWPEN(3)
      CALL PLCHAR(XL,YL,0.8*CSL,'Velocity just downstream of rotor',
     &            0.0,-1)
      YL = YL - 1.8*CSL
      IF(DUCT) THEN
        CALL PLCHAR(XL,YL,0.7*CSL,'V + v_axi+duct includes freestream',
     &              0.0,-1)
       ELSE
        CALL PLCHAR(XL,YL,0.7*CSL,'V + v_axi includes freestream',
     &              0.0,-1)
      ENDIF
      YL = YL - 1.8*CSL
      CALL PLCHAR(XL,YL,0.7*CSL,'Swirl includes rotor viscous drag',
     &            0.0,-1)
C
      CALL PLFLUSH
C
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = 1.0
      IF(WIND) THEN
       XYFAC(2) = -WFAC
      ELSE
       XYFAC(2) =  WFAC
      ENDIF     
C
      RETURN
      END ! UVIPLT3











