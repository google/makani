C***********************************************************************
C    Module:  xmodi.f
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

      SUBROUTINE MODI
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 COMAND, CONSTR, ANS
      CHARACTER*132 COMARG, PARARG
      CHARACTER*12 CHPARM(11)
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR, OK
C
C----------------------------------------------------------
C     Modify rotor
C----------------------------------------------------------
      PLFACD = 0.6
      PLFAC1 = 0.7
      PLFAC2 = 0.85
      XORG  = 0.15
      YORG  = 0.10
C
      CFACA = 1.0
      CFACB = 0.0
      TDEG  = 0.0
C
      GREEK = .FALSE.
C
 900  CONTINUE
      CALL ASKC('.MODI^',COMAND,COMARG)
C
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
      IF(COMAND.EQ.'    ') RETURN
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'BLAD') GO TO 10
      IF(COMAND.EQ.'MODB') GO TO 30
      IF(COMAND.EQ.'MODC') GO TO 32
      IF(COMAND.EQ.'SCAL') GO TO 34
      IF(COMAND.EQ.'TLIN') GO TO 36
      IF(COMAND.EQ.'RTIP') GO TO 40
      IF(COMAND.EQ.'RHUB') GO TO 42
      IF(COMAND.EQ.'RWAK') GO TO 44 
      IF(COMAND.EQ.'RAKE') GO TO 46 
      IF(COMAND.EQ.'XPAX') GO TO 48
      IF(COMAND.EQ.'CLPI') GO TO 50
      IF(COMAND.EQ.'CLPO') GO TO 60
      IF(COMAND.EQ.'OPTI') GO TO 80
      IF(COMAND.EQ.'PLOT') GO TO 90
      IF(COMAND.EQ.'ANNO') GO TO 92
      IF(COMAND.EQ.'HARD') GO TO 94
      IF(COMAND.EQ.'SIZE') GO TO 96
      WRITE(*,1000) COMAND
      GO TO 900
C
C---------------------------------------------------------------------
 10   IF(NINPUT.GE.1) THEN
       NBLDS = IINPUT(1)
      ELSE
       CALL ASKI('Enter new number of blades^',NBLDS)
      ENDIF
      GO TO 900
C---------------------------------------------------------------------
 30   CONTINUE
      CALL MODBE
      GO TO 900
C---------------------------------------------------------------------
 32   CONTINUE
      CALL MODCH
      GO TO 900
C---------------------------------------------------------------------
 34   IF(NINPUT.GE.2) THEN
       CFACA = RINPUT(1)
       CFACB = RINPUT(2)
      ELSE
       WRITE(*,*) ' '
       WRITE(*,*) '(Chord)new = (Chord)old * [A + B(r/R)]'
       CALL ASKR('Enter constant chord scaling factor A^',CFACA)
       CALL ASKR('Enter  linear  chord scaling factor B^',CFACB)
      ENDIF
C
      DO I=1, II
        CH(I) = CH(I) * (CFACA + CFACB*XI(I))
      ENDDO
      GO TO 900
C
C---------------------------------------------------------------------
 36   IF(NINPUT.GE.1) THEN
       TDEG = RINPUT(1)
      ELSE
       CALL ASKR('Tip angle change (deg)^',TDEG)
      ENDIF
C
      TRAD = TDEG * PI/180.
      DO I=1, II
        BETA(I)  = BETA(I)  + TRAD * XI(I)/XITIP
        BETA0(I) = BETA0(I) + TRAD * XI(I)/XITIP
      ENDDO
      GO TO 900
C
C--------------------------------------------------------------
 40   IF(NINPUT.GE.1) THEN
       RADNEW = RINPUT(1)
      ELSE
       RADNEW = RAD
       WRITE(*,1040) RADNEW
 1040  FORMAT(/1X,'Current tip radius =', F9.4)
       CALL ASKR('Enter new tip radius (m)^',RADNEW)
      ENDIF
      XI0 = XI0 * RAD / RADNEW
      XW0 = XW0 * RAD / RADNEW
      RAD = RADNEW
      CALL SETX
      CALL XWINIT
      GO TO 900
C
C--------------------------------------------------------------
 42   IF(NINPUT.GE.1) THEN
       RHUB = RINPUT(1)
      ELSE
       RHUB = XI0*RAD
       WRITE(*,1042) RHUB
 1042  FORMAT(/1X,'Current hub radius =', F9.4)
       CALL ASKR('Enter new hub radius (m)^',RHUB)
      ENDIF
      XI0 = RHUB / RAD
      CALL SETX
      CALL XWINIT
      GO TO 900
C
C--------------------------------------------------------------
 44   IF(NINPUT.GE.1) THEN
       RWAK = RINPUT(1)
      ELSE
       RWAK = XW0*RAD
       WRITE(*,1044) RWAK
 1044  FORMAT(/1X,'Current hub wake displacement radius =', F9.4)
       CALL ASKR('Enter new hub wake displacement radius (m)^',RWAK)
      ENDIF
      XW0 = RWAK / RAD
      CALL XWINIT
      GO TO 900
C
C--------------------------------------------------------------
 46   IF(NINPUT.GE.1) THEN
       RAKD = RINPUT(1)
      ELSE
       RAKD = RAKE*180./PI
       WRITE(*,1046) RAKD
 1046  FORMAT(/1X,'Current blade rake angle =', F9.4)
       CALL ASKR('Enter new blade rake angle (deg)^',RAKD)
      ENDIF
      RAKE = RAKD * PI/180.
      GO TO 900
C
C---------------------------------------------------------------------
 48   IF(NINPUT.GE.1) THEN
       XPITCH = RINPUT(1)
      ELSE
       WRITE(*,1048) XPITCH
 1048  FORMAT(/1X,'Current pitch-axis x/c location^', F9.4)
       CALL ASKR('Enter new pitch-axis x/c location^',XPITCH)
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C---- clip inner rotor radius
 50   IF(NINPUT.GE.1) THEN
       RADNEW = RINPUT(1)
      ELSE
       RADINR = XI0*RAD
       WRITE(*,1060) RADINR
 1050  FORMAT(/1X,'Current hub radius =', F9.4)
       CALL ASKR('Enter new hub radius (m)^',RADINR)
      ENDIF
C
      DO I = 1, II
        W0(I) = XI(I)
        W1(I) = CH(I)
        W3(I) = BETA(I)
      ENDDO
      CALL SPLINE(W1,W2,W0,II)
      CALL SPLINE(W3,W4,W0,II)
C
      XI0 = RADINR / RAD
      CALL SETX
      CALL XWINIT
C
      DO I = 1, II
ccc        CHOLD = CH(I)
ccc        BETOLD = BETA(I)
        CH(I)   = SEVAL(XI(I),W1,W2,W0,II)
        BETA(I) = SEVAL(XI(I),W3,W4,W0,II)
ccc        write(*,*) 'clipinr i,ch,beta ',i,chold,ch(i),betold,beta(i)
      ENDDO
      GO TO 900
C
C---------------------------------------------------------------------
C---- clip outer rotor radius
 60   IF(NINPUT.GE.1) THEN
       RADNEW = RINPUT(1)
      ELSE
       WRITE(*,1060) RAD
 1060  FORMAT(/1X,'Current tip radius =', F9.4)
       CALL ASKR('Enter new tip radius (m)^',RADNEW)
      ENDIF
C
      DO I = 1, II
        W0(I) = XI(I)
        W1(I) = CH(I)
        W3(I) = BETA(I)
      ENDDO
      CALL SPLINE(W1,W2,W0,II)
      CALL SPLINE(W3,W4,W0,II)
C
      RRAT = RAD/RADNEW
C
      XI0 = XI0 * RRAT
      XW0 = XW0 * RRAT
      RAD = RADNEW
      CALL SETX
      CALL XWINIT
C
      DO I = 1, II
        CH(I)   = SEVAL(XI(I)/RRAT,W1,W2,W0,II) * RRAT
        BETA(I) = SEVAL(XI(I)/RRAT,W3,W4,W0,II)
      ENDDO
      GO TO 900
C
C---------------------------------------------------------------------
C---- optimize twist
 80   CONTINUE
      TDES = TTOT
      PDES = PTOT
      IF(DEST) WRITE(*,*) 'Thrust will be held fixed'
      IF(DESP) WRITE(*,*) 'Power  will be held fixed'
C
C---- save current twist distribution
      DO I=1, II
        W5(I) = BETA(I)
        W6(I) = BETA0(I)
      ENDDO
      CONV = .FALSE.
C
C---- initial guess for 1/(ideal efficiency)
      EFFINV = PWAK/TWAK
C
C---- initial guess for optimum Gamma(r) using Betz-Prandtl criterion
      BLDS = FLOAT(NBLDS)
      ZETA = 2.0*(EFFINV - 1.0)
      SFAC = SQRT(1.0 + 1.0/ADW**2)
      DO I=1, II
        YY = XW(I)/ADV
        IF(DUCT) THEN
          F = 1.0
         ELSE
          ARG = MIN(20.0, 0.5*BLDS*(1.0 - XW(I)/XWTIP)*SFAC)
          EK = EXP(-ARG)
          FK = SQRT(1.0 - EK*EK)
          F = ATAN2(FK,EK)*2.0/PI
        ENDIF
        GAM(I) = F*YY*YY/(1.0+YY*YY) * (2.0*PI*ADV/BLDS) * ZETA*EFFINV
      ENDDO
C
C---- Converge MIL rotor (fixed chord, varying twist, varying CL case)
      CALL DESMIL(3)
C
      IF(CONV) THEN
C
C----- display blade twist changes and solution
       WRITE(*,2500) (I,XI(I),(BETA(I)-W5(I))*180.0/PI, I=1, II)
C
       BRLX = 1.0
       WRITE(*,1500) BRLX
 1500  FORMAT(/' Enter relaxation factor for blade angle changes:',F8.3)
       CALL READR(1,BRLX,ERROR)
C
       IF(BRLX .NE. 0.0) THEN
         DO I=1, II
           BETA(I)  = W5(I) + BRLX*(BETA(I) -W5(I))
           BETA0(I) = W6(I) + BRLX*(BETA0(I)-W6(I))
         ENDDO
C
         IF(DEST) THEN
           TSPEC = TTOT * (RHO*VEL**2*RAD**2)
           CALL APER(1,2,.TRUE.)
         ENDIF
         IF(DESP) THEN
           PSPEC = PTOT * (RHO*VEL**3*RAD**2)
           CALL APER(3,2,.TRUE.)
         ENDIF
         IF(CONV) CALL OUTPUT(LUWRIT)
       ELSE
         WRITE(*,*)
         WRITE(*,*) 'Blade geometry not modified.'
       ENDIF
C
      ELSE
C
C----- restored clobbered twist distribution
       DO I=1, II
         BETA(I)  = W5(I)
         BETA0(I) = W6(I)
       ENDDO
       WRITE(*,*) 'Blade twist distribution restored'
C
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
 90   IF(NINPUT.GE.1) THEN
       NPLOT = IINPUT(1)
      ELSE
       WRITE(*,2000)
       CALL ASKI('select plot number^',NPLOT)
      ENDIF
C
      IF    (NPLOT.EQ.0) THEN
       GO TO 900
C
C--- 3 view geometry plot of single blade
      ELSE IF(NPLOT.EQ.1) THEN
       CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC1*SIZE,LPLOT,LLAND)
       CALL PLOT(XORG,YORG,-3)
       CALL GEOPLT('ALUE')
C--- Geometry of all blades, axial view
      ELSE IF(NPLOT.EQ.2) THEN
        CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFACD*SIZE,LPLOT,.NOT.LLAND)
        CALL PLOT(0.175,0.175,-3)
        CALL PRPPLT
C--- Plot of operating point (Gam, CL, M, eff) + data
      ELSE IF(NPLOT.EQ.3) THEN
       CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
       CALL PLOT(XORG,YORG,-3)
       CALL CLPLT
C--- Combined geometry and operating point
      ELSEIF(NPLOT.EQ.4) THEN
       CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFACD*SIZE,LPLOT,.NOT.LLAND)
       CALL PLOT(0.175,0.075,-3)
       CALL GEOPLT('AL')
       CALL PLOTABS(0.0,0.0,-3)
       CALL PLOT(0.175,0.875,-3)
       CALL CLPLT
C--- Induced velocities on blade
      ELSE IF(NPLOT.EQ.7) THEN
       CALL UVIPLT
C--- Induced velocities immediately downstream of rotor
      ELSE IF(NPLOT.EQ.8) THEN
c       CALL UVIPLT2
       CALL UVIPLT3
C--- Velocity triangles
      ELSE IF(NPLOT.EQ.9) THEN
       CALL TRIPLT
C--- Imposed external slipstream velocities
      ELSE IF(NPLOT.EQ.10) THEN
       IF(NADD.LT.2) THEN
        WRITE(*,*) 'No slipstream profiles present'
        GO TO 900
       ENDIF
       CALL VELPLT
C--- Plot reference x,y data
      ELSE IF(NPLOT.EQ.11) THEN
       FNAME = ' '
       CALL REFPLT(FNAME, XYOFF(1),XYOFF(2),XYFAC(1),XYFAC(2),
     &             0.5*CSIZE, 1)
C--- Plot blade parameters vs r/R
      ELSE IF(NPLOT.EQ.12) THEN
        CALL PLOT_DATA(NAME)
C
      ELSE
       NINPUT = 0
       GO TO 90
C
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
 92   IF(LPLOT) THEN
       CALL ANNOT(1.2*CSIZE)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
 94   IF(LPLOT) THEN
       CALL PLEND
       CALL REPLOT(IDEVRP)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
 96   IF(NINPUT.GE.1) THEN
       SIZE = RINPUT(1)
      ELSE
       WRITE(*,*) 'Current plot-object size =', SIZE
       CALL ASKR('Enter new plot-object size^',SIZE)
      ENDIF
      GO TO 900
C.....................................................................
C
 1000 FORMAT(1X,A4,' command not recognized.' //
     &             '  Type "?" for list, <Return> to exit menu.')
 1100 FORMAT(
     &  /'   BLAD i  Set new number of blades'
     &  /'   MODC    Modify chord distribution'
     &  /'   MODB    Modify blade twist angle distribution'
     &  /'   SCAL rr Scale current chords'
     &  /'   TLIN r  Add linear blade twist (proportional to r/R)'
     &  /'   RTIP r  Change tip radius (preserve chord/R)'
     &  /'   RHUB r  Change hub radius'
     &  /'   RWAK r  Change hub wake displacement body radius'
     &  /'   RAKE r  Change blade rake angle'
     &  /'   XPAX r  Change pitch-axis x/c'
     & //'   CLPI r  Clip current prop at inner radius (preserve chord)'
     &  /'   CLPO r  Clip current prop at outer radius (preserve chord)'
     & //'   OPTI    Optimize blade twist angles for current planform'
     & //'   PLOT i  Plot various rotor parameters'
     &  /'   ANNO    Annotate current plot'
     &  /'   HARD    Hardcopy current plot'
     &  /'   SIZE r  Change plot-object size')
C 
 2000 FORMAT(/'  0   CANCEL'
     &       /'  1   Geometry'
     &       /'  2   Axial Geometry (all blades)'
     &       /'  3   Radial distributions for current case'
     &       /'  4   Radial distributions plus geometry'
     &       /'  7   Induced velocities on blade vs r/R'
     &       /'  8   Induced velocities in slipstream vs r/R'
     &       /'  9   Velocity triangles'
     &       /' 10   External slipstream velocity profiles'
     &       /' 11   Reference x,y data'
     &       /' 12   Plot blade data (Gam,CL,CD,etc) vs r/R')
C
 2500 FORMAT(/' Blade angle changes'
     &       /'   i    r/R   d(beta)', 100(/ 1X, I4, F7.3, F9.3) )
C               15  0.785  -10.345
C
      END ! MODI




      SUBROUTINE MODBE
      INCLUDE 'XROTOR.INC'
C------------------------------------------------
C     Takes user cursor input to modify 
C     current blade angle array.
C------------------------------------------------
      DIMENSION YLIMS(2)
      EXTERNAL PLCHAR,PLMATH
C
      RTD = 180./PI
      PLFAC = 0.95
      PLPAR = 1.20*PAR
      SH = 0.3*CSIZE
      NSPLT = 20
C
C---- work with temporary arrays
      BETMIN = BETA(1)*RTD
      BETMAX = BETA(1)*RTD
      DO I=1, II
        W1(I) = XI(I)
        W2(I) = BETA(I) * RTD
        BETMIN = MIN(BETA(I)*RTD,BETMIN)
        BETMAX = MAX(BETA(I)*RTD,BETMAX)
      ENDDO
      CALL SPLINE(W2,W3,W1,II)
C
      DY = 5.0
      DBET = BETMAX - BETMIN
      YLIMS(1) = BETMIN-0.25*DBET
      YLIMS(2) = BETMAX+0.25*DBET
      CALL PLTMOD(II,W1,W2,DY,YLIMS,
     &            PLFAC,PLPAR,NSPLT,XOFF,XSF,YOFF,YSF)
C
      XPLT = -5.0*CSIZE
      YPLT = PLPAR - 0.5*DY*YSF - 0.7*CSIZE
      CALL PLMATH(XPLT,YPLT,1.4*CSIZE,'b"',0.0,2)
C
C---- get new W2 array
      CALL CRSMOD(II,W1,W2,W3,
     &            XOFF,XSF,YOFF,YSF, SH, NSPLT,
     &            LSLOPE, IMOD1,IMOD2 )
C
C---- store blade angle changes
      DO I = IMOD1, IMOD2
        DBET = W2(I) * PI/180.0  -  BETA(I)
        BETA(I)  = BETA(I)  + DBET
        BETA0(I) = BETA0(I) + DBET
      ENDDO
C
      RETURN
      END



      SUBROUTINE MODCH
      INCLUDE 'XROTOR.INC'
C------------------------------------------------
C     Takes user cursor input to modify 
C     current blade chord array.
C------------------------------------------------
      DIMENSION YLIMS(2)
      EXTERNAL PLCHAR,PLMATH
C
      PLFAC = 0.95
      PLPAR = 1.20*PAR
      SH = 0.3*CSIZE
      NSPLT = 20
C
C---- work with temporary arrays
      CHMAX = CH(1)
      DO I=1, II
        W1(I) = XI(I)
        W2(I) = CH(I)
        CHMAX = MAX(CH(I),CHMAX)
      ENDDO
      CALL SPLINE(W2,W3,W1,II)
C
      DY = 0.01
      YLIMS(1) = 0.
      YLIMS(2) = 1.25*CHMAX
      CALL PLTMOD(II,W1,W2,DY,YLIMS,
     &            PLFAC,PLPAR,NSPLT,XOFF,XSF,YOFF,YSF)
C
      XPLT = -6.0*CSIZE
      YPLT = PLPAR - 0.5*DY*YSF - 0.7*CSIZE
      CALL PLCHAR(XPLT,YPLT,1.4*CSIZE,'c/R',0.0,3)
C
C---- get new W2 array
      CALL CRSMOD(II,W1,W2,W3,
     &            XOFF,XSF,YOFF,YSF, SH, NSPLT,
     &            LSLOPE, IMOD1,IMOD2 )
C
C---- store as blade chords
      DO I = IMOD1, IMOD2
        CH(I) = W2(I)
      ENDDO
C
      RETURN
      END



      SUBROUTINE PLTMOD(N,X,Y,DYMIN,YLIMS,
     &                  PLFAC,PLPAR,NSPLT,XOFF,XSF,YOFF,YSF)
      INCLUDE 'XROTOR.INC'
      DIMENSION X(N),Y(N)
      DIMENSION YLIMS(2)
C-------------------------------------------------------------
C     Plots Y(X) array with grid overlay, assumes 0<=X<=1
C
C     user can optionally specify Y limits(min,max) or autoscaling 
C     in arrays YLIMS(1) = ymin (999. for autoscale on ymin)
C               YLIMS(2) = ymax (999. for autoscale on ymax)
C
C     Returns the scaling factors and offsets.
C
C     Intended for use with user-driven cursor interaction.
C-------------------------------------------------------------
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      YMIN = Y(1)
      YMAX = Y(1)
      DO I=2, II
        YMIN = MIN(YMIN,Y(I))
        YMAX = MAX(YMAX,Y(I))
      ENDDO
      IF(YLIMS(1).NE.999.) YMIN = YLIMS(1) 
      IF(YLIMS(2).NE.999.) YMAX = YLIMS(2) 
C
      IF(ABS(YMAX-YMIN).EQ.0.) THEN
        IF(YMAX.NE.0.) THEN
          YMIN = 0.5*YMAX
          YMAX = 1.5*YMAX
         ELSE
          YMAX = YMIN + 1.0
        ENDIF
      ENDIF
C
      CALL SCALIT(1,YMAX,YMIN,YFAC)
C
      YDEL = MAX( 1.0/(5.0*YFAC) , DYMIN )
      YMIN = YDEL*(AINT(YMIN/YDEL + 1000.8) - 1001.0)
      YMAX = YDEL*(AINT(YMAX/YDEL + 1001.2) - 1000.0)
C
C---- increase y range if it is too narrow for useful editing
      IF((YMAX-YMIN) .LT. 0.5*(YMAX+YMIN)) THEN
        YMAX = YMAX + YDEL
        YMIN = YMIN - YDEL
      ENDIF
      IF((YMAX-YMIN) .LT. 0.5*(YMAX+YMIN)) THEN
        YMAX = YMAX + YDEL
        YMIN = YMIN - YDEL
      ENDIF
C
      DYMIN = YDEL
C
      YSF = PLPAR / (YMAX-YMIN)
      YOFF = YMIN
C
      XSF = 1.0
      XOFF = 0.0
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(1.0,1.0,-3)
C
      CALL GETCOLOR(ICOL0)
C
      CS = 1.4*CSIZE
      CALL NEWPEN(2)
C
      CALL XAXIS(0.0,0.0,1.0,0.2    , 0.0,0.2, CSIZE,1)
      XPLT = 0.5 - 1.2*CS
      YPLT =     - 2.5*CS
      CALL PLCHAR(XPLT,YPLT,CS,'r/R',0.0,3)
C
      CALL YAXIS(0.0,0.0,PLPAR,YDEL*YSF, YMIN,YDEL, CSIZE,-2)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = 10
       NYG = INT( (YMAX-YMIN)/(0.5*YDEL) + 0.01 )
       CALL PLGRID(0.0,0.0, NXG,0.1, NYG,0.5*YDEL*YSF, LMASK2 )
      ENDIF
C
      SH = 0.4*CSIZE
      CALL NEWCOLORNAME('blue')
      CALL XYSYMB(II,X,Y,XOFF,XSF,YOFF,YSF,SH,1)
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWPEN(2)
      CALL XYLINE(II,X,Y,XOFF,XSF,YOFF,YSF,1)
C
      CALL PLFLUSH
C
      RETURN
      END
