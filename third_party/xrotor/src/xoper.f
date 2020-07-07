C***********************************************************************
C    Module:  xoper.f
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

      SUBROUTINE OPER
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 COMAND, ANS
      CHARACTER*132 COMARG, ANSARG
      CHARACTER*1 CHKEY
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
C---------------------------------------------
C     Run rotor at arbitrary operating points
C---------------------------------------------
      PLFAC1 = 0.7
      PLFAC2 = 0.8
      PLFACD = 0.6
      XORG = 0.15
      YORG = 0.10
C
      GREEK = .FALSE.
C
 900  CONTINUE
      CALL ASKC('.OPER^',COMAND,COMARG)
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
      IF(COMAND.EQ.'    ') THEN
       CALL CLRZOOM
       RETURN
      ENDIF
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'FORM') GO TO 2
      IF(COMAND.EQ.'TERS') GO TO 4
      IF(COMAND.EQ.'HARD') GO TO 6
      IF(COMAND.EQ.'SIZE') GO TO 8
      IF(COMAND.EQ.'ANNO') GO TO 9
      IF(COMAND.EQ.'DISP') GO TO 10
      IF(COMAND.EQ.'NAME') GO TO 15
      IF(COMAND.EQ.'WRIT') GO TO 20
      IF(COMAND.EQ.'DUCT') GO TO 22
      IF(COMAND.EQ.'VRAT') GO TO 24
      IF(COMAND.EQ.'PLOT') GO TO 30
      IF(COMAND.EQ.'ATMO') GO TO 35
      IF(COMAND.EQ.'VELO') GO TO 38
      IF(COMAND.EQ.'ANGL') GO TO 40
      IF(COMAND.EQ.'ADVA') GO TO 42
      IF(COMAND.EQ.'RPM ') GO TO 45
      IF(COMAND.EQ.'THRU') GO TO 50
      IF(COMAND.EQ.'TORQ') GO TO 60
      IF(COMAND.EQ.'POWE') GO TO 70
      IF(COMAND.EQ.'ASEQ') GO TO 81
      IF(COMAND.EQ.'RSEQ') GO TO 82
      IF(COMAND.EQ.'BSEQ') GO TO 83
      IF(COMAND.EQ.'VSEQ') GO TO 84
      IF(COMAND.EQ.'CLRC') GO TO 90
      IF(COMAND.EQ.'ADDC') GO TO 92
      IF(COMAND.EQ.'CPUT') GO TO 94
      IF(COMAND.EQ.'CGET') GO TO 96
      IF(COMAND.EQ.'CASE') GO TO 97
      IF(COMAND.EQ.'LIST') GO TO 98
C
      IF(COMAND.EQ.'N')    GO TO 72
      IF(COMAND.EQ.'ITER') GO TO 75
      IF(COMAND.EQ.'INIT') GO TO 76
      IF(COMAND.EQ.'REIN') GO TO 78
C
C--- Hack to check ADW equation sensitivity, get rid of this later... HHY
      IF(COMAND.EQ.'ADW') THEN
        WRITE(*,*) 'Current ADW factor =', ADWFCTR
        CALL ASKR('Enter new ADW factor^',ADWFCTR)
        GO TO 900
      ENDIF
C
C--- Read or use engine rpm/power line file
      IF(COMAND.EQ.'PVAR') GO TO 79
C
      IF(COMAND.EQ.'Z   ') THEN
       CALL USETZOOM(.TRUE.,.TRUE.)
       CALL REPLOT(IDEV)
       GO TO 900
      ENDIF
      IF(COMAND.EQ.'U   ') THEN
       CALL CLRZOOM
       CALL REPLOT(IDEV)
       GO TO 900
      ENDIF
C
      WRITE(*,1050) COMAND
      GO TO 900
C
C---------------------------------------------------------------------
C--- Select options for slipstream and velocity calculation
 2    CONTINUE
      WRITE(*,3)
      CALL ASKC('.FORM^',COMAND,COMARG)
C
      IF(COMAND.EQ.'GRAD') THEN
        VRTX = .FALSE.
        FAST = .TRUE.
       ELSEIF(COMAND.EQ.'POT') THEN
        VRTX = .FALSE.
        FAST = .FALSE.
       ELSEIF(COMAND.EQ.'VRTX') THEN
        VRTX = .TRUE.
       ELSEIF(COMAND.EQ.'WAKE') THEN
        FREE = .NOT.FREE
       ELSEIF(COMAND.EQ.' ') THEN
        GO TO 900
      ENDIF
C
      IF(VRTX) THEN
       WRITE(*,*)'Discrete Vortex Formulation selected'
      ELSE
       IF(FAST) THEN
        WRITE(*,*) 'Graded Momentum Formulation selected'
       ELSE
        WRITE(*,*)'Potential Formulation selected'
       ENDIF
      ENDIF
C
      IF(FREE) THEN
        WRITE(*,*)'Self-deforming wake selected'
       ELSE
        WRITE(*,*)'Rigid wake selected'
      ENDIF
      GO TO 2
C
 3    FORMAT(
     &  /' Select options for calculation of slipstream velocities'
     &  /'   GRAD     use Graded Momentum       Formulation '
     &  /'   POT      use Potential (Goldstein) Formulation '
     &  /'   VRTX     use discrete Vortex Wake  Formulation '
     &  /'   WAKE     Toggle between rigid and self-deforming wake')
C
C
C---------------------------------------------------------------------
C--- Output data on blade stations with each case (verbose)
    4 TERSE = .NOT.TERSE
      IF(TERSE)      WRITE(*,*)'Terse output selected'
      IF(.NOT.TERSE) WRITE(*,*)'Verbose output selected'
      GO TO 900
C
C---------------------------------------------------------------------
C--- Hardcopy current plot
    6 IF(LPLOT) THEN
       CALL PLEND
       CALL REPLOT(IDEVRP)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change plot size
    8 IF(NINPUT.GE.1) THEN
       SIZE = RINPUT(1)
      ELSE
       WRITE(*,*) 'Current plot size =', SIZE
       CALL ASKR('Enter new plot size^',SIZE)
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Annotate plot
    9 IF(LPLOT) THEN
       CALL ANNOT(1.2*CSIZE)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Display current prop operating point data 
   10 CALL OUTPUT(LUWRIT)
ccc      CALL CPROJ
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change case name
   15 NAME = COMARG
      IF(NAME(1:1).EQ.' ')
     &  CALL ASKS('Enter case name (32 characters max)^',NAME)
      GO TO 900
C
C---------------------------------------------------------------------
C--- Write current prop operating point data to file
   20 IF(COMARG(1:1).NE.' ') SAVFIL = COMARG
      CALL OPFILE(LUSAVE,SAVFIL)
      CALL OUTPUT(LUSAVE)
      CLOSE(LUSAVE)
      GO TO 900
C
C--------------------------------------------------------------
 22   DUCT = .NOT.DUCT
      IF(DUCT) THEN
       WRITE(*,*) 'Duct option selected'
       IF(NINPUT.GE.1) THEN
        URDUCT = RINPUT(1)
       ELSE
        CALL ASKR('Enter Aexit/Aprop for duct^',URDUCT)
       ENDIF
      ELSE
       WRITE(*,*) 'Free-tip option selected'
       URDUCT = 1.0
      ENDIF
      GO TO 900
C
C--------------------------------------------------------------
 24   IF(DUCT) THEN
       IF(NINPUT.GE.1) THEN
        URDUCT = RINPUT(1)
       ELSE
        CALL ASKR('Enter Aexit/Aprop for duct^',URDUCT)
       ENDIF
      ELSE
       WRITE(*,*) '*** Select duct option first'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Plot stuff
   30 IF(NINPUT.GE.1) THEN
       NPLOT = IINPUT(1)
      ELSE
       WRITE(*,2000)
       NPLOT = 3
       CALL ASKI('select plot number^',NPLOT)
      ENDIF
C
      IF(NPLOT.EQ.0) THEN
       GO TO 900
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
C--- Data for stored cases (vs r/R)
      ELSE IF(NPLOT.EQ.5) THEN
       CALL ACLPLT
C--- Case sequence parameters
      ELSE IF(NPLOT.EQ.6) THEN
       CALL CASPLT
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
       GO TO 30
C
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change altitude
   35 IF(NINPUT.GE.1) THEN
       ALT = RINPUT(1)
      ELSE
       CALL ASKR('flight altitude (km)^',ALT)
      ENDIF
      CALL ATMO(ALT,VSO,RHO,RMU)
      CALL FLOSHO(LUWRIT, VSO, RHO, RMU)
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change flight velocity
   38 VELOLD = VEL
      IF(NINPUT.GE.1) THEN
       VEL = RINPUT(1)
      ELSE
       CALL ASKR('flight speed (m/s)^',VEL)
      ENDIF
C--- Change CT,CQ,CP to give same thrust,torque,power
      THR  = TTOT *(RHO*VELOLD**2*RAD**2)
      TTOT = THR / (RHO*VEL**2*RAD**2)
      TRQ  = QTOT *(RHO*VELOLD**2*RAD**3)
      QTOT = TRQ / (RHO*VEL**2*RAD**3)
      PWR  = PTOT *(RHO*VELOLD**3*RAD**2)
      PTOT = PWR / (RHO*VEL**3*RAD**2)
      CONV = .FALSE.
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change blade pitch
   40 IF(NINPUT.GE.1) THEN
       DELB = RINPUT(1)
      ELSE
       CALL ASKR('angle change (deg)^',DELB)
      ENDIF
      DO I=1, II
        BETA(I)  = BETA(I)  + DELB*PI/180.
        BETA0(I) = BETA0(I) + DELB*PI/180.
      ENDDO
      CONV = .FALSE.
      GO TO 900      
C
C---------------------------------------------------------------------
C--- Specify advance ratio and solve
   42 IF(NINPUT.GE.1) THEN
       ADV = RINPUT(1)
      ELSE
       CALL ASKR('advance ratio     ^',ADV)
      ENDIF
      CONV = .FALSE.
      CALL APER(4,2,LOPRINI)
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Specify RPM and solve
   45 IF(NINPUT.GE.1) THEN
       RPM = RINPUT(1)
      ELSE
       RPM  = VEL/(RAD*ADV*PI/30.)
       CALL ASKR('rpm               ^',RPM)
      ENDIF
      ADV = VEL / (RAD*RPM*PI/30.)
      CONV = .FALSE.
      CALL APER(4,2,LOPRINI)
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Specify thrust and solve
   50 IF(NINPUT.GE.1) THEN
       TSPEC = RINPUT(1)
      ELSE
       TSPEC = TTOT * (RHO*VEL**2*RAD**2)
       CALL ASKR('thrust (N)        ^',TSPEC)
      ENDIF
      RPM = VEL / (RAD*ADV*PI/30.0)
      WRITE(*,1530) RPM
   51 CALL ASKC('fix Pitch / fix Rpm ( P/R )?^',
     &          ANS,ANSARG)
      IF(ANS.NE.'R' .AND. ANS.NE.'P') GO TO 51
C
      CONV = .FALSE.
      BSAV = BETA(II)
      IF(ANS.EQ.'P') CALL APER(1,2,LOPRINI)
      IF(ANS.EQ.'R') THEN
       CALL ASKR('rpm:^',RPM)
       ADV = VEL / (RAD*RPM*PI/30.0)
       CALL APER(1,1,LOPRINI)
      ENDIF
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
C---- Check for valid blade angle change 
      IF(ANS.NE.'P') THEN
       IF(CONV) THEN
C----- convergence was achieved: show blade angle change incurred
        WRITE(*,1550) DBETA*180.0/PI
       ELSE
C----- convergence failed: restore clobbered blade angles
        DO I=1, II
         BETA(I)  = BETA(I)  - DBETA
         BETA0(I) = BETA0(I) - DBETA
        ENDDO
       ENDIF
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Specify torque and solve
   60 IF(NINPUT.GE.1) THEN
       QSPEC = RINPUT(1)
      ELSE
       QSPEC = QTOT * (RHO*VEL**2*RAD**3)
       CALL ASKR('torque (N-m)      ^',QSPEC)
      ENDIF
      RPM = VEL / (RAD*ADV*PI/30.0)
      WRITE(*,1530) RPM
   61 CALL ASKC('fix Pitch / fix Rpm ( P/R )?^',
     &          ANS,ANSARG)
      IF(ANS.NE.'R' .AND. ANS.NE.'P') GO TO 61
C
      CONV = .FALSE.
      IF(ANS.EQ.'P') CALL APER(2,2,LOPRINI)
      IF(ANS.EQ.'R') THEN
       CALL ASKR('rpm:^',RPM)
       ADV = VEL / (RAD*RPM*PI/30.0)
       CALL APER(2,1,LOPRINI)
      ENDIF
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
C---- Check for valid blade angle change 
      IF(ANS.NE.'P') THEN
       IF(CONV) THEN
C----- convergence was achieved: show blade angle change incurred
        WRITE(*,1550) DBETA*180.0/PI
       ELSE
C----- convergence failed: restore clobbered blade angles
        DO I=1, II
         BETA(I)  = BETA(I)  - DBETA
         BETA0(I) = BETA0(I) - DBETA
        ENDDO
       ENDIF
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Specify power and solve
   70 IF(NINPUT.GE.1) THEN
       PSPEC = RINPUT(1)
      ELSE
       PSPEC = PTOT * (RHO*VEL**3*RAD**2)
       CALL ASKR('Power (W)         ^',PSPEC)
      ENDIF
      RPM = VEL / (RAD*ADV*PI/30.0)
      WRITE(*,1530) RPM
   71 CALL ASKC('fix pitch / fix rpm ( P/R )?^',
     &          ANS,ANSARG)
      IF(ANS.NE.'R' .AND. ANS.NE.'P') GO TO 71
C
      CONV = .FALSE.
      IF(ANS.EQ.'P') CALL APER(3,2,LOPRINI)
      IF(ANS.EQ.'R') THEN
       CALL ASKR('rpm:^',RPM)
       ADV = VEL / (RAD*RPM*PI/30.0)
       CALL APER(3,1,LOPRINI)
      ENDIF
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
C---- Check for valid blade angle change 
      IF(ANS.NE.'P') THEN
       IF(CONV) THEN
C----- convergence was achieved: show blade angle change incurred
        WRITE(*,1550) DBETA*180.0/PI
       ELSE
C----- convergence failed: restore clobbered blade angles
        DO I=1, II
         BETA(I)  = BETA(I)  - DBETA
         BETA0(I) = BETA0(I) - DBETA
        ENDDO
       ENDIF
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Change number of radial points for blade stations
 72   CONTINUE
      IF(LROTOR) THEN
       IISAV = II
       DO I = 1, IISAV
         W1(I) = XI(I)
         W2(I) = CH(I)
         W4(I) = BETA(I)
         W6(I) = UBODY(I)
         W8(I) = CLDES(I)
       ENDDO
       CALL SPLINE(W2,W3,W1,II)
       CALL SPLINE(W4,W5,W1,II)
       CALL SPLINE(W6,W7,W1,II)
       CALL SPLINE(W8,W9,W1,II)
      ENDIF
C
 73   CALL ASKI('Enter new number of radial points^',II)
      IF(II.GT.IX) THEN
       WRITE(*,*)
       WRITE(*,*) 'Maximum number is', IX
       GO TO 73
      ENDIF
C
      IINF = II + II/2
      CALL SETX
      IF(LROTOR) THEN
       DO I = 1, II
         CH(I)    = SEVAL(XI(I),W2,W3,W1,IISAV)
         BETA(I)  = SEVAL(XI(I),W4,W5,W1,IISAV)
         UBODY(I) = SEVAL(XI(I),W6,W7,W1,IISAV)
         CLDES(I) = SEVAL(XI(I),W8,W9,W1,IISAV)
         BETA0(I) = BETA(I)
       ENDDO
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Set max number or iterations for nonlinear solution
 75   IF(NINPUT.GE.1) THEN
       NITERA = IINPUT(1)
      ELSE
       CALL ASKI('Max number of iterations^',NITERA)
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Toggle initialization flag 
 76   LOPRINI = .NOT.LOPRINI
      IF(LOPRINI) THEN
       WRITE(*,*) 'Analysis case will be initialized'
      ELSE
       WRITE(*,*) 'Analysis case will not be initialized'
      ENDIF
      GO TO 900
C
C---------------------------------------------------------------------
C--- Reinitialize operating point
 78   CALL REINIT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Read or use engine rpm/power line file
 79   IF(LPWRVAR .AND. NPWRVAR.GT.0) THEN
        WRITE(*,*) ' '
        WRITE(*,*) 'Current RPM/Power Engine Line'
        DO L = 1, NPWRVAR
          WRITE(*,*) L,RPMVAR(L),PWRVAR(L)
        END DO
        WRITE(*,*) ' '
      ENDIF     
C
      LU = 12
      FNAME = COMARG
      IF(FNAME(1:1).EQ.' ') THEN
        CALL ASKS('Enter power/rpm filename^',FNAME)
      ENDIF
      IF(FNAME(1:1).NE.' ') THEN
        OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=795)
        CALL GETPVAR(LU,IX,NPWRVAR,RPMVAR,PWRVAR)
        WRITE(*,*) ' '
        WRITE(*,*) 'RPM/Power Engine Line'
        DO L = 1, NPWRVAR
          WRITE(*,*) L,RPMVAR(L),PWRVAR(L)
          PWRVAR(L) = PWRVAR(L)
        END DO
        WRITE(*,*) ' '
        CALL SPLINA(PWRVAR,XPWRVAR,RPMVAR,NPWRVAR)
        CLOSE(LU)
        LPWRVAR = .TRUE.
      ENDIF     
C
C
C--- Use the engine rpm/power to define operating point
      IF(LPWRVAR) THEN
 791  CALL ASKC('fix Pitch / fix Rpm / fix Velocity ( P/R/V )?^',
     &          ANS,ANSARG)
      IF(ANS.EQ.' ') GO TO 900
      IF(ANS.NE.'R' .AND. ANS.NE.'P' .AND. ANS.NE.'V') GO TO 791
C
      CONV = .FALSE.
      IF(ANS.EQ.'P') CALL APER(5,2,LOPRINI)
      IF(ANS.EQ.'R') THEN
       CALL ASKR('rpm:^',RPM)
       ADV = VEL / (RAD*RPM*PI/30.0)
       CALL APER(5,1,LOPRINI)
      ENDIF
      IF(ANS.EQ.'V') THEN
       CALL ASKR('vel:^',VEL)
       ADV = VEL / (RAD*RPM*PI/30.0)
       CALL APER(5,2,LOPRINI)
      ENDIF
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
      IF(CONV) CALL OUTPUT(LUWRIT)
C---- Was the pitch changed?
      IF(ANS.EQ.'R') THEN
       IF(CONV) THEN
C----- convergence was achieved: show blade angle change incurred
        WRITE(*,1550) DBETA*180.0/PI
       ELSE
C----- convergence failed: restore clobbered blade angles
        DO I=1, II
          BETA(I)  = BETA(I)  - DBETA
          BETA0(I) = BETA0(I) - DBETA
        ENDDO
       ENDIF
      ENDIF
      GO TO 900
      ENDIF
C
 795  NF = INDEX(FNAME,' ') - 1
      WRITE(*,*) 'OPEN error on file  ',FNAME(1:NF)
      GO TO 900
C
C---------------------------------------------------------------------
C--- Do sequence of advance ratios 
 81   WRITE(*,*) ' '
      WRITE(*,*) 'Sequence of advance ratios...'
      CALL SETCAS(1,NINPUT,RINPUT)
      CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
      CALL CASPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Do sequence of RPMs 
 82   WRITE(*,*) ' '
      WRITE(*,*) 'Sequence of RPMs...'
      CALL SETCAS(2,NINPUT,RINPUT)
      CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
      CALL CASPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Do sequence of pitch angles 
 83   WRITE(*,*) ' '
      WRITE(*,*) 'Sequence of blade angles...'
      CALL SETCAS(3,NINPUT,RINPUT)
      CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
      CALL CASPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Do sequence of velocities 
 84   WRITE(*,*) ' '
      WRITE(*,*) 'Sequence of velocity with fixed pitch or RPM...'
      CALL SETCAS(4,NINPUT,RINPUT)
      CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
      CALL CASPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- Save current operating point to case arrays
 90   NCASE = 0
      KCASE = 0
      GO TO 900
C
 92   IF(NCASE.GE.ICASX) THEN
       WRITE(*,*) 'Case arrays too small.  Increase ICASX.'
       GO TO 900
      ENDIF
C
      NCASE = NCASE+1
      CASPAR(1,NCASE) = ADV
      CASPAR(2,NCASE) = VEL
      CASPAR(3,NCASE) = BETA(II)
      CASPAR(4,NCASE) = ALT
      CASPAR(5,NCASE) = RHO
      CASPAR(6,NCASE) = RMU
      CASPAR(7,NCASE) = VSO
      CASPAR(8,NCASE) = 999.
      CASPAR(9,NCASE) = 999.
      CASPAR(10,NCASE)= 999.
      CASPAR(11,NCASE)= 999.
      GO TO 900
C
C---------------------------------------------------------------------
C--- Write case accumulation arrays to file
 94   IF(NCASE.LE.0) THEN
       WRITE(*,*)
       WRITE(*,*) 'No cases saved'
       GO TO 900
      ENDIF
C
      LU = 12
      FNAME = COMARG
      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter case save filename^',FNAME)
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=945)
      WRITE(*,*) 'File exists.  Overwrite?  Y'
      READ (*,1000) CHKEY
      IF(INDEX('Nn',CHKEY) .NE. 0) THEN
                CLOSE(LU)
        GO TO 900
      ELSE
        REWIND LU
        GO TO 946
      ENDIF
C
 945  OPEN(LU,FILE=FNAME,STATUS='UNKNOWN',ERR=94)
 946  CALL SHOCAS(LU,NPARX,NCASE,CASPAR,RAD,NAME)
      CLOSE(LU)
      GO TO 900
C
C---------------------------------------------------------------------
C--- Read case accumulation arrays from saved file
 96   CONTINUE
      LU = 12
      FNAME = COMARG
      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter case save filename^',FNAME)
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=965)
      CALL GETCAS(LU,NPARX,NCASE,CASPAR)
      CLOSE(LU)
      CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
      GO TO 900
C
 965  NF = INDEX(FNAME,' ') - 1
      WRITE(*,*) 'OPEN error on file  ',FNAME(1:NF)
      GO TO 900
C
C---------------------------------------------------------------------
C--- Rerun case operating point
 97   IF(NCASE.LE.0) THEN
       WRITE(*,*)
       WRITE(*,*) 'No cases saved'
       GO TO 900
      ENDIF
C
      IF(NINPUT.GE.1) THEN
       ICASE = IINPUT(1)
      ELSE
       CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
       ICASE = 0
       CALL ASKI('Select case number (0 to cancel)^',ICASE)
      ENDIF
      IF(ICASE.LE.0) GO TO 900
      IF(ICASE.GT.NCASE) THEN
       NINPUT = 0
       GO TO 97
      ENDIF
C
      ADV = CASPAR(1,ICASE)
      VEL = CASPAR(2,ICASE)
      BET = CASPAR(3,ICASE)
      ALT = CASPAR(4,ICASE)
      RHO = CASPAR(5,ICASE)
      RMU = CASPAR(6,ICASE)
      VSO = CASPAR(7,ICASE)
      POW = CASPAR(8,ICASE)
      THR = CASPAR(9,ICASE)
      TRQ = CASPAR(10,ICASE)
      EFF = CASPAR(11,ICASE)
C
      DELB = BET - BETA(II)
      DO I=1, II
        BETA(I)  = BETA(I)  + DELB
        BETA0(I) = BETA0(I) + DELB
      ENDDO
C
      CONV = .FALSE.
      CALL APER(4,2,.TRUE.)
C
      IF(CONV) THEN
       CASPAR(8,ICASE) = PTOT*RHO*VEL**3*RAD**2
       CASPAR(9,ICASE) = TTOT*RHO*VEL**2*RAD**2
       CASPAR(10,ICASE)= QTOT*RHO*VEL**2*RAD**3
       CASPAR(11,ICASE)= TTOT/PTOT
      ENDIF
C
      IF(CONV) CALL OUTPUT(LUWRIT)
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC2*SIZE,LPLOT,LLAND)
      CALL PLOT(XORG,YORG,-3)
      CALL CLPLT
      GO TO 900
C
C---------------------------------------------------------------------
C--- List rotor dimensional data ?
 98   CONTINUE
      DO I=1, II
        WRITE(*,*) XI(I), NBLDS*GAM(I)*RAD*VEL, VIND(3,I)*VEL,
     &             NBLDS*GAM(I)/(4.0*PI*VIND(3,I)*XI(I))
      ENDDO
      GO TO 900

C.......................................................................
C
 1000 FORMAT(A)
 1050 FORMAT(1X,A4,' command not recognized.' //
     &             '  Type "?" for list, <Return> to exit menu.')
 1100 FORMAT(
     &  /'   ADVA r   Prescribe advance ratio'
     &  /'   RPM  r   Prescribe rpm'
     &  /'   THRU r   Prescribe thrust'
     &  /'   TORQ r   Prescribe torque'
     &  /'   POWE r   Prescribe power'
     & //'   ASEQ rrr Calculate case sequence of advance ratios'
     &  /'   RSEQ rrr Calculate case sequence of rpms'
     &  /'   BSEQ rrr Calculate case sequence of blade angles'
     &  /'   VSEQ rrr Calculate case sequence of speeds at fixed pitch'
     &  /'   CLRC     Clear case accumulator'
     &  /'   ADDC     Add current point point to case accumulator'
     &  /'   CPUT f   Write current case accumulator to file'
     &  /'   CGET f   Read cases from file'
     &  /'   CASE i   Select case'
     & //'   ATMO r   Set fluid properties from standard atmosphere'
     &  /'   VELO r   Set or change flight speed'
     &  /'   ANGL r   Change blade pitch angle'
     &  /'   PVAR f   Enter and use engine rpm/power line'
     & //'   FORM     Select slipstream and velocity formulation'
     & //'   NAME s   Set or change case name'
     &  /'   WRIT f   Write current operating point to disk file'
     &  /'   DISP     Display current operating state'
     &  /'   INIT     Initialize next analysis case'
     &  /'   REIN     Re-initialize prop to known operating state'
     &  /'   TERS     Toggle between terse and verbose output'
     &  /'   ITER i   Change max number of Newton iterations'
     &  /'   N    i   Change number of radial points'
     & //'   PLOT i   Plot various rotor parameters'
     &  /'   ANNO     Annotate plot'
     &  /'   HARD     Hardcopy current plot'
     &  /'   SIZE r   Change plot-object size')
 1530 FORMAT(/' Current rpm:', F9.2)
 1550 FORMAT(' Blade angle changed',F7.3,' degrees')
C
 2000 FORMAT(/'  0   CANCEL'
     &       /'  1   Geometry'
     &       /'  2   Axial Geometry (all blades)'
     &       /'  3   Radial distributions for current case'
     &       /'  4   Radial distributions plus geometry'
     &       /'  5   Radial distributions for all cases'
     &       /'  6   Case sequence parameters'
     &       /'  7   Induced velocities on blade vs r/R'
     &       /'  8   Induced velocities in slipstream vs r/R'
     &       /'  9   Velocity triangles'
     &       /' 10   External slipstream velocity profiles'
     &       /' 11   Reference x,y data'
     &       /' 12   Plot blade data (Gam,CL,CD,etc) vs r/R')
C
      END ! OPER



      SUBROUTINE GETPVAR(LU,NDIM,N,XRPM,XPWR)
      DIMENSION XPWR(NDIM),XRPM(NDIM)
      CHARACTER*1 DUMMY
C
 1000 FORMAT(A)
      READ(LU,1000) DUMMY
C
      DO I=1, 12345
        READ(LU,*,END=11,ERR=99) XX,YY
        XRPM(I) = XX
        XPWR(I) = YY
      ENDDO
 11   CONTINUE
      N = I-1
      RETURN
C
 99   WRITE(*,*) 'File read error'
      N = 0
      RETURN
      END



      SUBROUTINE SHOCAS(LU,NDIM,N,PAR,RAD,NAME)
      DIMENSION PAR(0:NDIM,*)
      CHARACTER NAME*(*)
C
      IF(NDIM.LT.11) THEN
        WRITE(*,*) 'Error in SHOCAS: NDIM too small for PAR array'
        RETURN
      ENDIF
C
      PI = 4.0*ATAN(1.0)
C
      WRITE(LU,900) NAME
      WRITE(LU,1000)
      DO I=1, N
        ADV = PAR(1,I)
        VEL = PAR(2,I)
        BET = PAR(3,I)*180.0/PI
        ALT = PAR(4,I)
        RHO = PAR(5,I)
        RMU = PAR(6,I)*1.0E5
        VSO = PAR(7,I)
        CONVFLG = PAR(8,I)
        POW = PAR(8,I)*0.001
        THR = PAR(9,I)
        TRQ = PAR(10,I)
        EFF = PAR(11,I)
        RPM = VEL/(RAD*ADV) * 30.0/PI
        IF(CONVFLG.EQ.999.0) THEN
          WRITE(LU,1200) I, ADV, BET, VEL, RPM, RHO, RMU, VSO, ALT
         ELSE
          WRITE(LU,1200) I, ADV, BET, VEL, RPM, RHO, RMU, VSO, ALT,
     &                   POW, THR, TRQ, EFF
        ENDIF
      ENDDO
      RETURN
C
  900 FORMAT(A)
C
C        1         2         3         4         5         6         7         8         9         0         1         2         3         4         5         6         7
C23456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
CIIXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGG
C  n         V/wR         Btip            V          rpm          rho       mu*1e5       Vsound            h        P(kW)         T(N)       Q(N-m)          eff',
C
 1000 FORMAT(
     & '  n'
     & '         V/wR         Btip            V          rpm'
     & '          rho       mu*1e5       Vsound            h'
     & '        P(kW)         T(N)       Q(N-m)          eff'
     & /160('-'))
 1200 FORMAT(I3,12(1X,G12.5))
c  
c 1000 FORMAT( '  n   V/wR   Btip      V       rpm ',
c     &        '    rho   mu*1e5   Vsound    h  ',
c     &        '       P(kW)        T(N)         Q(N-m)      eff',
c     &       /' --  ------  -----  ------- ------- ',
c     &        ' -----  -------  -------  ------ ',
c     &        '  -----------  -----------  -----------  -----')
c 1200 FORMAT(I3,1X,F7.3,1X,F6.2,1X,F7.2,1X,F8.1,1X,F6.3,1X,F8.3,
c     &       1X,F8.2,1X,F7.2,2X,G12.5,1X,G12.5,1X,G12.5,1X,F6.3)
c
c--- new format (old one ran out of sig. digits, lets remove the 80 column limit!
c  n   V/wR   Btip     V       rpm     rho    mu*1e5   Vsound    h        P(kW)        T(N)         Q(N-m)      eff
c --  ------  -----  ------  -------  -----  -------  -------  -----   -----------  -----------  -----------  -----
c 12 x0.2345 x14.25 x123.23 x10000.0 x1.225 x123.123 x1234.12 x123.12 x123456.1234 x123456.1234 x123456.1234 x0.111
ciii fffffff ffffff fffffff ffffffff ffffff ffffffff ffffffff fffffff gggggggggggg gggggggggggg gggggggggggg ffffff
C
c
c---------------------------------------------------------------------------
c 1000 FORMAT(/'  n   V/wR   Btip     V     rpm',
c     &        '      rho     P(kW)    T(N)     Q(N-m)   eff',
c     &       /' --  -----  -----  ------  -------',
c     &        '  -----  --------  -------  -------  -----')
c 1200 FORMAT(I3,1X,F6.3,1X,F6.2,1X,F6.2,1X,F8.1,1X,F6.3,
c     &       1X,F9.4,1X,F8.1,1X,F8.1,1X,F6.3)
c
c  n   V/wR   Btip     V     rpm      rho     P(kW)     T(N)     Q(N-m)   eff
c --  -----  -----  ------  -------  -----  --------  -------  -------  -----
c 12 x0.234 x14.25 x123.23 x10000.0 x1.225 x123.1234 x11000.0 x11125.0 x0.111
ciii ffffff ffffff fffffff ffffffff ffffff fffffffff ffffffff ffffffff ffffff
c---------------------------------------------------------------------------
c
c---------------------------------------------------------------------------
c 1000 FORMAT(
c     & /'  n    V/wR    Btip     V      rpm  ',
c     &  '    rho     mu*1e5  Vsound     h  '
c     & /'  --   ------  -----  ------  -------',
c     &  '  -------  -------  ------   -----')
cC          12   0.2345  14.25  123.23  10000.0
cC             1.225  115.000  1000.0   25.00
c 1200 FORMAT(
c     &   I3, F9.4,    F7.2,   F8.2,     F9.1,
c     &       F9.3,    F9.3,     F8.1,  F8.2  )
c---------------------------------------------------------------------------
C
c---------------------------------------------------------------------------
c 1000 FORMAT(
c     & /'  n    V/wR    Btip     V     rpm  ',
c     &  '  rho     P(W)     T(N)     Q(N-m)',
c     & /'  --   ------  -----  ------  ------',
c     &  '  -----  -------  ------   ------')
C
c 1200 FORMAT(
c     &   I3, F9.4,    F7.2,   F8.2,     F9.1,
c     &       F7.4,  F10.2,   F9.3,  F9.3  )
c---------------------------------------------------------------------------
C
      END


      SUBROUTINE GETCAS(LU,NDIM,NCAS,PAR)
      DIMENSION PAR(0:NDIM,*), A(16)
      CHARACTER DUMMY*1, LINE*128, CNAME*32
      LOGICAL ERROR
C
      IF(NDIM.LT.11) THEN
        WRITE(*,*) 'Error in GETCAS: NDIM too small for PAR array'
        RETURN
      ENDIF
C
      PI = 4.0*ATAN(1.0)
C
 1000 FORMAT(A)
      READ(LU,1000) CNAME
cc      WRITE(*,*) 'Case name: ',CNAME
      READ(LU,1000) DUMMY
      READ(LU,1000) DUMMY
C
      DO I=1, 12345
ccc        READ(LINE,ERR=99) IDUM,ADV,BET,VEL,RPM,RHO,RMU,VSO,ALT,
ccc     &                    POW,THR,TRQ,EFF
        READ(LU,1000,END=11) LINE
        N = 13
        CALL GETFLT(LINE,A,N,ERROR)
        IF(ERROR) GO TO 99
        ADV = A(2)
        BET = A(3)*PI/180.0
        VEL = A(4)
        RHO = A(6)
        RMU = A(7)/1.0E5
        VSO = A(8)
        ALT = A(9)
        POW = 999.0
        THR = 999.0
        TRQ = 999.0
        EFF = 999.0
        IF(N.EQ.13) THEN
         POW = A(10)*1000.0
         THR = A(11)
         TRQ = A(12)
         EFF = A(13)
        ENDIF
C--- Set parameters for cases
        PAR(1,I) = ADV
        PAR(2,I) = VEL
        PAR(3,I) = BET
        PAR(4,I) = ALT
        PAR(5,I) = RHO
        PAR(6,I) = RMU
        PAR(7,I) = VSO
        PAR(8,I) = POW
        PAR(9,I) = THR
        PAR(10,I)= TRQ
        PAR(11,I)= EFF
      ENDDO
 11   CONTINUE
      NCAS = I-1
      RETURN
C
 99   WRITE(*,*) 'File read error'
      NCAS = 0
      RETURN
      END



      SUBROUTINE SETCAS(ITYPE,NINPUT,RINPUT)
      INCLUDE 'XROTOR.INC'
      DIMENSION RINPUT(*)
C---------------------------------------------------
C     Sets operating parameters over a range 
C     of parameters of type ITYPE where
C       ITYPE    Parameter for range
C         1      Advance ratio
C         2      RPM
C         3      Blade angle
C         4      Velocity with fixed pitch
Cxxxx     5      Velocity with fixed RPM
C---------------------------------------------------
      CHARACTER*1 ANS, ANS4*4
      LOGICAL YES
C
c      WRITE(*,*)
c      WRITE(*,*) 'Overwrite or Append  to case accumulator?  O'
c      READ (*,1000) ANS
c 1000 FORMAT(A)
c      IF(INDEX('Aa',ANS) .EQ. 0) NCASE = 0
C
C
      IF(NCASE.GT.0) THEN
        WRITE(*,*)
        WRITE(*,*) 'Appending to current case accumulator...'
      ENDIF
C
      KCASE = 0
C
C---------------------------------------------------------------------
C--- Sequence of advance ratio
      IF(ITYPE.EQ.1) THEN
        KCASE = 1
C
        IF    (NINPUT.GE.3) THEN
         ADV1 = RINPUT(1)
         ADV2 = RINPUT(2)
         DADV = RINPUT(3)
        ELSEIF(NINPUT.GE.2) THEN
         ADV1 = RINPUT(1)
         ADV2 = RINPUT(2)
         DADV = 999.
         CALL ASKR('Enter advance ratio increment  ^',DADV)
        ELSEIF(NINPUT.GE.1) THEN
         ADV1 = RINPUT(1)
         ADV2 = 999.
         CALL ASKR('Enter last  advance ratio value^',ADV2)
         DADV = 999.
         CALL ASKR('Enter advance ratio increment  ^',DADV)
        ELSE
         ADV1 = 999.
         CALL ASKR('Enter first advance ratio value^',ADV1)
         ADV2 = 999.
         CALL ASKR('Enter last  advance ratio value^',ADV2)
         DADV = 999.
         CALL ASKR('Enter advance ratio increment  ^',DADV)
        ENDIF
        IF(ADV1.EQ.ADV2) RETURN
        DADV = SIGN(DADV,ADV2-ADV1)
        NP = 1
        IF(DADV .NE. 0.0) NP = INT((ADV2-ADV1)/DADV + 0.5) + 1
        IF(NP.LE.0) RETURN
C
C--- Check for use of rpm/power relationship to set power
        YES = .FALSE.
        XANS = 0.
        IF(LPWRVAR) CALL ASKL('Use engine rpm/power line ?^',YES)
        IF(YES) XANS = 100.0
C
        IF(NCASE+NP .GT. ICASX) THEN
         WRITE(*,*) 'Limiting number of cases to array limit:', ICASX
         NP = ICASX - NCASE
        ENDIF
C
        DO IP=1, NP
          NCASE = NCASE + 1
          CASPAR(0,NCASE) = XANS + FLOAT(KCASE)
          CASPAR(1,NCASE) = ADV1 + DADV*FLOAT(IP-1)
          CASPAR(2,NCASE) = VEL
          CASPAR(3,NCASE) = BETA(II)
          CASPAR(4,NCASE) = ALT
          CASPAR(5,NCASE) = RHO
          CASPAR(6,NCASE) = RMU
          CASPAR(7,NCASE) = VSO
          CASPAR(8,NCASE) = 999.
          CASPAR(9,NCASE) = 999.
          CASPAR(10,NCASE)= 999.
          CASPAR(11,NCASE)= 999.
        ENDDO
C
C---------------------------------------------------------------------
C--- Sequence of RPM
      ELSEIF(ITYPE.EQ.2) THEN
        KCASE = 2
C
        IF    (NINPUT.GE.3) THEN
         RPM1 = RINPUT(1)
         RPM2 = RINPUT(2)
         DRPM = RINPUT(3)
        ELSEIF(NINPUT.GE.2) THEN
         RPM1 = RINPUT(1)
         RPM2 = RINPUT(2)
         DRPM = 999.
         CALL ASKR('Enter rpm increment  ^',DRPM)
        ELSEIF(NINPUT.GE.1) THEN
         RPM1 = RINPUT(1)
         RPM2 = 999.
         CALL ASKR('Enter last  rpm value^',RPM2)
         DRPM = 999.
         CALL ASKR('Enter rpm increment  ^',DRPM)
        ELSE
         RPM1 = 999.
         CALL ASKR('Enter first rpm value^',RPM1)
         RPM2 = 999.
         CALL ASKR('Enter last  rpm value^',RPM2)
         DRPM = 999.
         CALL ASKR('Enter rpm increment  ^',DRPM)
        ENDIF
        IF(RPM1.EQ.RPM2) RETURN
        DRPM = SIGN(DRPM,RPM2-RPM1)
        NP = 1
        IF(DRPM .NE. 0.0) NP = INT((RPM2-RPM1)/DRPM + 0.5) + 1
        IF(NP.LE.0) RETURN
C
C--- Check for use of rpm/power relationship to set power
        YES = .FALSE.
        XANS = 0.
        IF(LPWRVAR) CALL ASKL('Use engine rpm/power line ?^',YES)
        IF(YES) XANS = 100.0
C
        ANS = ' '
        CALL ASKS('Fix power P or thrust T or blade pitch A ?^',ANS)
        CALL LC2UC(ANS)
        IF(ANS.EQ.'T') XANS = XANS + 1000.0
        IF(ANS.EQ.'Q') XANS = XANS + 2000.0
        IF(ANS.EQ.'P') XANS = XANS + 3000.0
C
        IF(NCASE+NP .GT. ICASX) THEN
         WRITE(*,*) 'Limiting number of cases to array limit:', ICASX
         NP = ICASX - NCASE
        ENDIF
C
        DO IP=1, NP
          NCASE = NCASE + 1
          RPM = RPM1 + DRPM*FLOAT(IP-1)
          CASPAR(0,NCASE) = XANS + FLOAT(KCASE)
          CASPAR(1,NCASE) = VEL/(RPM*RAD) * 30.0/PI
          CASPAR(2,NCASE) = VEL
          CASPAR(3,NCASE) = BETA(II)
          CASPAR(4,NCASE) = ALT
          CASPAR(5,NCASE) = RHO
          CASPAR(6,NCASE) = RMU
          CASPAR(7,NCASE) = VSO
          CASPAR(8,NCASE) = 999.
          CASPAR(9,NCASE) = 999.
          CASPAR(10,NCASE)= 999.
          CASPAR(11,NCASE)= 999.
        ENDDO
C
C---------------------------------------------------------------------
C--- Sequence of blade angle
      ELSEIF(ITYPE.EQ.3) THEN
        KCASE = 3
C
        IF    (NINPUT.GE.3) THEN
         BET1 = RINPUT(1)
         BET2 = RINPUT(2)
         DBET = RINPUT(3)
        ELSEIF(NINPUT.GE.2) THEN
         BET1 = RINPUT(1)
         BET2 = RINPUT(2)
         DBET = 999.
         CALL ASKR('Enter tip angle increment   (deg) ^',DBET)
        ELSEIF(NINPUT.GE.1) THEN
         BET1 = RINPUT(1)
         BET2 = 999.
         CALL ASKR('Enter last  tip angle value (deg) ^',BET2)
         DBET = 999.
         CALL ASKR('Enter tip angle increment   (deg) ^',DBET)
        ELSE
         BET1 = 999.
         CALL ASKR('Enter first tip angle value (deg) ^',BET1)
         BET2 = 999.
         CALL ASKR('Enter last  tip angle value (deg) ^',BET2)
         DBET = 999.
         CALL ASKR('Enter tip angle increment   (deg) ^',DBET)
        ENDIF
        IF(BET1.EQ.BET2) RETURN
        DBET = SIGN(DBET,BET2-BET1)
        NP = 1
        IF(DBET .NE. 0.0) NP = INT((BET2-BET1)/DBET + 0.5) + 1
        IF(NP.LE.0) RETURN
C
C--- Check for use of rpm/power relationship to set power
        YES = .FALSE.
        XANS = 0.
        IF(LPWRVAR) CALL ASKL('Use engine rpm/power line ?^',YES)
        IF(YES) XANS = 100.0
C
        IF(NCASE+NP .GT. ICASX) THEN
         WRITE(*,*) 'Limiting number of cases to array limit:', ICASX
         NP = ICASX - NCASE
        ENDIF
C
        DO IP=1, NP
          NCASE = NCASE + 1
          BET = BET1 + DBET*FLOAT(IP-1)
          CASPAR(0,NCASE) = XANS + FLOAT(KCASE)
          CASPAR(1,NCASE) = ADV
          CASPAR(2,NCASE) = VEL
          CASPAR(3,NCASE) = BET * PI/180.0
          CASPAR(4,NCASE) = ALT
          CASPAR(5,NCASE) = RHO
          CASPAR(6,NCASE) = RMU
          CASPAR(7,NCASE) = VSO
          CASPAR(8,NCASE) = 999.
          CASPAR(9,NCASE) = 999.
          CASPAR(10,NCASE)= 999.
          CASPAR(11,NCASE)= 999.
        ENDDO
C
C---------------------------------------------------------------------
C--- Sequence of velocities 
      ELSEIF(ITYPE.EQ.4) THEN
        KCASE = 4
C
        IF    (NINPUT.GE.3) THEN
         VEL1 = RINPUT(1)
         VEL2 = RINPUT(2)
         DVEL = RINPUT(3)
        ELSEIF(NINPUT.GE.2) THEN
         VEL1 = RINPUT(1)
         VEL2 = RINPUT(2)
         DVEL = 999.
         CALL ASKR('Enter speed increment   (m/s) ^',DVEL)
        ELSEIF(NINPUT.GE.1) THEN
         VEL1 = RINPUT(1)
         VEL2 = 999.
         CALL ASKR('Enter last  speed value (m/s) ^',VEL2)
         DVEL = 999.
         CALL ASKR('Enter speed increment   (m/s) ^',DVEL)
        ELSE
         VEL1 = 999.
         CALL ASKR('Enter first speed value (m/s) ^',VEL1)
         VEL2 = 999.
         CALL ASKR('Enter last  speed value (m/s) ^',VEL2)
         DVEL = 999.
         CALL ASKR('Enter speed increment   (m/s) ^',DVEL)
        ENDIF
        IF(VEL1.EQ.VEL2) RETURN
        DVEL = SIGN(DVEL,VEL2-VEL1)
        NP = 1
        IF(DVEL .NE. 0.0) NP = INT((VEL2-VEL1)/DVEL + 0.5) + 1
        IF(NP.LE.0) RETURN
C
C--- Check for use of rpm/power relationship to set power
        YES = .FALSE.
        XANS = 0.
        IF(LPWRVAR) CALL ASKL('Use engine rpm/power line ?^',YES)
        IF(YES) XANS = 100.0
C
C--- What do we hold constant, pitch or rpm?
 20     ANS4 = 'CS'
        CALL ASKS('FP fixed-pitch or CS constant-speed^',ANS4)
        CALL LC2UC(ANS4)
        IF(ANS4.NE.'CS' .AND. ANS4.NE.'FP') GO TO 20
        IF(ANS4.EQ.'CS') THEN
          RPM  = VEL/(RAD*ADV*PI/30.)
          CALL ASKR('Enter constant rpm value^',RPM)
          ADV = VEL / (RAD*RPM*PI/30.)
          KCASE = 5
          IF(XANS.NE.100.0) THEN
            IF(PSPEC.LE.0.0 .AND. PTOT.GT.0.0) 
     &         PSPEC = PTOT * (RHO*VEL**3*RAD**2)
            CALL ASKR('Enter constant power value^',PSPEC)
          ENDIF
        ENDIF
C
        IF(NCASE+NP .GT. ICASX) THEN
         WRITE(*,*) 'Limiting number of cases to array limit:', ICASX
         NP = ICASX - NCASE
        ENDIF
C
        DO IP=1, NP
          NCASE = NCASE + 1
          VVEL  = VEL1 + DVEL*FLOAT(IP-1)
          CASPAR(0,NCASE) = XANS + FLOAT(KCASE)
          CASPAR(1,NCASE) = VVEL * ADV/VEL
          CASPAR(2,NCASE) = VVEL
          CASPAR(3,NCASE) = BETA(II)
          CASPAR(4,NCASE) = ALT
          CASPAR(5,NCASE) = RHO
          CASPAR(6,NCASE) = RMU
          CASPAR(7,NCASE) = VSO
          CASPAR(8,NCASE) = 999.
          CASPAR(9,NCASE) = 999.
          CASPAR(10,NCASE)= 999.
          CASPAR(11,NCASE)= 999.
        ENDDO
C
      ENDIF
C
      RETURN
      END ! SETCAS



      SUBROUTINE APER(ISPEC,ICON,LINIT)
      INCLUDE 'XROTOR.INC'
      LOGICAL LINIT
C-------------------------------------------
C     Sets reasonable initial circulation.
C     Converges arbitrary operating point.
C
C     ISPEC controls the quantity used as a target quantity
C       ISPEC = 1   Drive thrust to TSPEC
C       ISPEC = 2   Drive torque to QSPEC
C       ISPEC = 3   Drive power  to PSPEC
C       ISPEC = 4   Fix advance ratio to current value
C       ISPEC = 5   Drive to power specified by RPM (engine power-RPM line)
C     ICON controls the constrained quantity
C       ICON = 1    Advance ratio(rpm) fixed
C       ICON = 2   Blade pitch fixed
C     LINIT is flag for initialization of rotor condition
C-------------------------------------------
C
C--- Initialize circulations if requested
      IF(LINIT) THEN
ccc        WRITE(*,*) 'APINIT called...'
        CALL APINIT
      ENDIF
ccc      CALL PLOT_DATA(NAME)
C
ccc      WRITE(*,*) 'Before APITER ADV,ADW ',adv, adw
      CALL APITER(ISPEC,ICON)
C
      IF(.NOT.CONV) THEN
       WRITE(*,*)
       WRITE(*,*) 'Iteration limit exceeded'
       WRITE(*,*) 'Gres Fres Ares =', GRESMX, FRESMX, ARESMX
      ENDIF
C
      RETURN
      END ! APER


      SUBROUTINE APINIT
      INCLUDE 'XROTOR.INC'
C---------------------------------------------------------
C     Sets reasonable initial circulation.
C     Initial circulations are set w/o induced effects
C     An iteration is done using the self-induced velocity
C     from graded momentum theory to converge an approximate
C     wake advance ratio
C----------------------------------------------------------
C
      DATA NITERG / 10 /
C
      BLDS = FLOAT(NBLDS)
      DBETA = 0.0
C
      UDUCT     = 0.0
      VADUCT_VA = 1.0
      IF(DUCT) THEN
        UDUCT = URDUCT-1.0
        VADUCT_VA = 2.0*URDUCT
      ENDIF
      ADW = ADV * (1.0 + UDUCT)
C
C======================================================================
C---- Initialize section circulation neglecting induced velocity
      TSUM  = 0.
      DO I = 1, II
        UTOT = URDUCT + UBODY(I)
        CALL UVADD(XI(I),WA,WT)
C
        SI = UTOT      + WA
        CI = XI(I)/ADV - WT
C
        WSQ = CI*CI + SI*SI
        W = SQRT(WSQ)
        PHI = ATAN2(SI,CI)
C
        ALFA = BETA(I) - PHI
        REY = CH(I)*ABS(W) * RHO*VEL*RAD/RMU
        CALL GETCLCDCM(I,ALFA,W,REY,
     &                 CL(I),CL_AL,CL_W,
     &                 CLMAX,CLMIN,DCLSTALL,STALL(I),
     &                 CD(I),CD_ALF,CD_W,CD_REY,
     &                 CM(I),CM_AL,CM_W)
C
        GAM(I) = 0.5*CL(I)*W*CH(I)
        TSUM = TSUM + BLDS*GAM(I)*CI*DXI(I)
cc        write(8,997) 'i,alfa,cl,gam,tsum ',i,alfa,cl(i),gam(i),tsum
      ENDDO
 997  format(A,' ',i4,5(1x,f10.5))
C
C---- use momentum theory estimate of axial velocity to set wake adv. ratio
      VHSQ = 0.5*TSUM/PI
      VHSQ = MAX( VHSQ , -0.25 )
      ADW = ADV * 0.5*(1.0 + SQRT(1.0 + 4.0*VHSQ))
C
ccc      WRITE(*,*) 'APINIT noVind TSUM,ADW ',TSUM,ADW
ccc      CALL PLOT_DATA(NAME)
C
C---- recalculate Vtan using new GAM values
      CALL VCALC
CC    GO TO 101
C
C======================================================================
C---- Refine the initial guess with a graded-momentum theory estimate
C     Use momentum theory to estimate axial induced velocity to drive 
C     equation for wake advance ratio
C
      DO 100 ITERG = 1, NITERG
C
        CALL GRADMO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
C
        TSUM  = 0.
        T_ADW = 0.
C
        DCLMAX = 0.
        RLXMIN = 1.0
C
        DO 10 I = 1, II
C
C--- Redefine VT and VA to diagonal self-influences
          VT     = VIND_GAM(3,I,I)*GAM(I)
          VT_GAM = VIND_GAM(3,I,I)
          VT_ADW = VIND_ADW(3,I)
C
          VA     = VIND_GAM(1,I,I)*GAM(I)
          VA_GAM = VIND_GAM(1,I,I)
          VA_ADW = VIND_ADW(1,I)
C
C------ include duct effect on freestream and induced axial velocity
          UDUCT     = 0.0
          VADUCT_VA = 1.0
          IF(DUCT) THEN
            UDUCT = URDUCT-1.0
            VADUCT_VA = 2.0*URDUCT
          ENDIF
C
          UTOT = 1.0 + UDUCT + UBODY(I)
          CALL UVADD(XI(I),WA,WT)
C
          CI     =  XI(I)/ADV - WT  -  VT
          CI_ADV = -XI(I)/ADV**2
          CI_VT  =                  -  1.0
C
          SI     = UTOT + WA  +  VA*VADUCT_VA
          SI_VA  =                  VADUCT_VA
ccc        SI     = UTOT + WA  +  VA
ccc        SI_VA  =               1.0
C
          WSQ = CI*CI + SI*SI
          W = SQRT(WSQ)
          W_ADV = (CI*CI_ADV            )/W
          W_VT  = (CI*CI_VT             )/W
          W_VA  = (            SI*SI_VA )/W
C
          PHI = ATAN2(SI,CI)
          P_ADV = (          - SI*CI_ADV)/WSQ
          P_VT  = (          - SI*CI_VT )/WSQ
          P_VA  = (CI*SI_VA             )/WSQ
C
          ALFA   = BETA(I) - PHI
          AL_VT  =         - P_VT
          AL_VA  =         - P_VA
C
          REY = CH(I)*ABS(W) * RHO*VEL*RAD/RMU
          CALL GETCLCDCM(I,ALFA,W,REY,
     &                   CL(I),CL_AL,CL_W,
     &                   CLMAX,CLMIN,DCLSTALL,STALL(I),
     &                   CD(I),CD_ALF,CD_W,CD_REY,
     &                   CM(I),CM_AL,CM_W)
ccc          write(*,*) 'iterg,i,cl ',iterg,i,cl(i)
C
C-------- Res( CL( AL W ) , W , GAM )
          REZ  = CH(I)*CL(I)*W - 2.0*GAM(I)
          Z_CL = CH(I)      *W
          Z_W  = CH(I)*CL(I)
          Z_G  =               - 2.0
C
C-------- Res( AL( VT ADW ) , W( VT ADW ) , GAM )
          Z_AL = Z_CL*CL_AL
          Z_W  = Z_CL*CL_W   +  Z_W
C
C-------- Res( VT(GAM ADW) , ADW , GAM )
          Z_VT  = Z_W*W_VT   + Z_AL*AL_VT
          Z_VA  = Z_W*W_VA   + Z_AL*AL_VA
C
C-------- Res( ADW , GAM )
          Z_ADW = Z_VT*VT_ADW + Z_VA*VA_ADW
          Z_G   = Z_VT*VT_GAM + Z_VA*VA_GAM + Z_G
C
          DELG = -REZ / Z_G
          DCL = 2.0*DELG/(CH(I)*W)
C
C---- Apply limiter to GAM update based on CL change
          RLX = 1.0
          IF(RLX*ABS(DCL) .GT. 0.2) THEN
            IF(DCL.NE.0.0) THEN
              RLX = MIN(RLX,0.2/ABS(DCL))
ccc        write(*,998) 'APER CL limiter i,rlx,dcl,cl',i,rlx,dcl,cl(i)
            ENDIF

          ENDIF
 998    format(a,2x,i5,3(2x,F12.5))
C
          IF(ABS(DCL) .GT. ABS(DCLMAX)) DCLMAX = DCL
          IF(ABS(RLX) .LT. RLXMIN)      RLXMIN = RLX
C
          GAM(I) = GAM(I) + RLX*DELG
C-------- dREZ = Z_G*dG + Z_ADW*dADW = 0
          G_ADW = -Z_ADW / Z_G
C
ccc Forces for raked blade corrected for COS of rake angle
c          COSR = COS(RAKE)
          COSR = 1.0
C
          TSUM = TSUM + BLDS*GAM(I)*CI*DXI(I)*COSR
          T_G  =        BLDS       *CI   *DXI(I)*COSR
          T_VT =        BLDS*GAM(I)*CI_VT*DXI(I)*COSR
          T_ADW = T_ADW + (T_G + T_VT*VT_GAM)*G_ADW 
     &                  +        T_VT*VT_ADW
 10     CONTINUE
C
C---- Momentum theory estimate of induced axial velocity
        VHSQ   = 0.5*TSUM/PI
        VHSQ   = MAX( VHSQ , -0.2499 )
        VHSQ_T = 0.5     /PI
C
        REZ   = ADW  -  ADV * 0.5*(1.0        + SQRT(1.0 + 4.0*VHSQ))
        Z_ADW = 1.0  - ADV/SQRT(1.0+4.0*VHSQ) * VHSQ_T*T_ADW
cc      Z_ADW = 1.0
        IF(Z_ADW.EQ.0.0) WRITE(*,*) 'APINIT Z_ADW ',Z_ADW
C
        DADW = -REZ / Z_ADW
        DADW = MIN( DADW , 10.0*ADW )
        DADW = MAX( DADW , -0.9*ADW )
        ADW = ADW + DADW
C
        IF(RLXMIN.LT.0.2) THEN
ccc          WRITE(*,*) 'APINIT filtering GAM'
          CALL FILTER(GAM,0.2*II,II)
        ENDIF
ccc        WRITE(*,*) 'APINIT Vind iter,TSUM,ADW ',ITERG,TSUM,ADW
ccc        WRITE(*,*) 'APINIT ADW,DADW,DCLMAX ',ADW,DADW,DCLMAX
C
        IF(ABS(DCLMAX) .LT. 0.001) GO TO 101
C
 100  CONTINUE
ccc      WRITE(*,*) 'APINIT No convergence'
C
 101  RETURN
      END


      SUBROUTINE APITER(ISPEC,ICON)
C-------------------------------------------------------
C     Converges arbitrary performance operating point
C
C     ISPEC controls the quantity used as a target quantity

C       ISPEC = 1   Drive thrust to TSPEC
C       ISPEC = 2   Drive torque to QSPEC
C       ISPEC = 3   Drive power  to PSPEC
C       ISPEC = 4   Fix advance ratio to current value
C       ISPEC = 5   Drive to power specified by RPM (engine power-RPM line)
C
C     ICON controls the constrained quantity
C       ICON = 1    Advance ratio(rpm) fixed
C       ICON = 2    Blade pitch fixed
C-------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      DIMENSION CLMAX(IX), CLMIN(IX), DCLSTALL(IX)
C
C---- convergence tolerance
      DATA EPS / 1.0E-07 /
C
      K1 = II+1
      K2 = II+2
      K3 = II+3
      WRITE(*,2000)
C
      DO 1000 ITER=1, MAX( NITERA , 1 )
C
C---- if wake advance ratio changed, recalculate Vtan influence coefficients
      IF(FREE .OR. ITER.EQ.1) THEN
        IF(FAST) THEN
          CALL GRADMO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 1
        ELSEIF(.NOT.VRTX) THEN
          CALL HELICO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 2
        ELSEIF(VRTX) THEN
          CALL VRTXCO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 3
        ENDIF
      ENDIF
C
C---- recalculate Vtan
      CALL VCALC
C
C---- recalculate wake radius array and Vwak
      CALL SETXW
C
C---- recalculate thrust, power, and sensitivities for current solution
      CALL TPQ(1)
C
C---- initialize max residuals
      GRESMX = 0.
      FRESMX = 0.
      ARESMX = 0.
C
      DO J=1, K3
        Q(K2,J) = 0.
      ENDDO
C
C---- The wake advance ratio equation is only approximate, normally the
C     tangential induced velocity is ignored (inconsistent with a rigid 
C     wake with constant wake advance ratio).  This calculates a factor 
C     to compensate for the Vt term at one (representative) radial station
C
      DO I = 1, II
        IF(XI(I).GT.0.75) GO TO 40
      END DO
 40   I75 = I
      CALL CSCALC(I75,UTOT,WA,WT,
     &              VT75,VT_ADW,
     &              VA75,VA_ADW,
     &              VD75,VD_ADW,
     &              CI75,CI_ADV,CI_VT,
     &              SI75,       SI_VA,
     &              W75,  W_ADV, W_VT, W_VA,
     &              PHI75,P_ADV, P_VT, P_VA)
C---- Factor for OMEG*R-VT correction to wake advance ratio
      ADVFACT = 1.0 / (1.0 - ADV*VT75/XI(I75))
ccc      WRITE(*,*) 'ADV factor ',ADVFACT
C---- Set to 1.0 for now... HHY
      ADVFACT = 1.0
C
      IF(FREE) THEN
C----- Set up equation to converge wake advance ratio based on 
C      average axial velocity consistent with basic momentum theory
C
C---- Use "equivalent" prop thrust and power 
       DQ(K2) =  ADWFCTR*ADW*TWAK/PWAK  -  ADV*ADVFACT
       Z_TW   =  ADWFCTR*ADW     /PWAK    
       Z_PW   = -ADWFCTR*ADW*TWAK/PWAK**2 
       DO J=1, II
         Q(K2,J) = Z_TW*TW_GAM(J) + Z_PW*PW_GAM(J)
       END DO
       Q(K2,K1) = Z_TW*TW_ADV + Z_PW*PW_ADV - ADVFACT
       Q(K2,K2) = Z_TW*TW_ADW + Z_PW*PW_ADW + ADWFCTR*TWAK/PWAK
       ARESMX = MAX( ARESMX , ABS(DQ(K2)/ADV) )
      ELSE
C----- specify zero change of wake advance ratios
       DQ(K2) = 0.
       Q(K2,K2) = 1.0
      ENDIF
C
C---- go over stations, enforcing Gamma-CL relation at real prop
      DO 100 I=1, II
C
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,             SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
C
        ALFA   = BETA(I) - PHI
        AL_DBE =  1.0
        AL_P   = -1.0
C
        REY = CH(I)*ABS(W) * RHO*VEL*RAD/RMU
        CALL GETCLCDCM(I,ALFA,W,REY,
     &                 CL(I),CL_AL,CL_W,
     &                 CLMAX(I),CLMIN(I),DCLSTALL(I),STALL(I),
     &                 CD(I),CD_ALF,CD_W,CD_REY,
     &                 CM(I),CM_AL,CM_W)
C
C------ Enforce local Gamma-CL relation
        DQ(I) = CH(I)*CL(I)*W - 2.0*GAM(I)             ! Residual
        Z_CL  = CH(I)      *W
        Z_W   = CH(I)*CL(I)
C
        Z_GI  =               - 2.0
        Z_VT  = Z_CL*(CL_AL*AL_P*P_VT  + CL_W*W_VT )  + Z_W*W_VT
        Z_VA  = Z_CL*(CL_AL*AL_P*P_VA  + CL_W*W_VA )  + Z_W*W_VA
        Z_ADV = Z_CL*(CL_AL*AL_P*P_ADV + CL_W*W_ADV)  + Z_W*W_ADV
        Z_DBE = Z_CL*(CL_AL*AL_DBE                 )
C
        DO J=1, II
          Q(I,J) = Z_VT*VIND_GAM(3,I,J) 
     &           + Z_VA*VIND_GAM(1,I,J)                ! dRes/dGamj
        ENDDO
        Q(I,I) = Q(I,I) + Z_GI                         ! dRes/dGami
        Q(I,K1) = Z_ADV                                ! dRes/dAdv
        Q(I,K2) =         Z_VT*VT_ADW + Z_VA*VA_ADW    ! dRes/dAdw
        Q(I,K3) = Z_DBE                                ! dRes/dBeta
C
        GRESMX = MAX( GRESMX , ABS( DQ(I)/(0.1*W) ) )
C
  100 CONTINUE
C
C---- equivalent prop will be used to define inviscid thrust
      IF(ISPEC.EQ.1) THEN
C----- drive thrust to specified value
       T_SPEC = TSPEC / (RHO*VEL**2*RAD**2)
       DQ(K1) = TWAK + TVIS - T_SPEC
       DO J=1, II
         Q(K1,J) = TW_GAM(J) + TV_GAM(J)
       ENDDO
       Q(K1,K1) = TW_ADV + TV_ADV
       Q(K1,K2) = TW_ADW + TV_ADW
       Q(K1,K3) =          TV_DBE
C
       FRESMX = MAX( FRESMX , ABS(DQ(K1)) )
C
      ELSE IF(ISPEC.EQ.2) THEN
C----- drive torque (= PTOT*ADV) to specified value
       Q_SPEC = QSPEC / (RHO*VEL**2*RAD**3)
       DQ(K1) = (PWAK + PVIS)*ADV - Q_SPEC
       DO J=1, II
         Q(K1,J) = (PW_GAM(J) + PV_GAM(J))*ADV
       ENDDO
       Q(K1,K1) = (PW_ADV + PV_ADV)*ADV  +  PWAK + PVIS
       Q(K1,K2) = (PW_ADW + PV_ADW)*ADV
       Q(K1,K3) = (         PV_DBE)*ADV
C
       FRESMX = MAX( FRESMX , ABS(DQ(K1)) )
C
      ELSE IF(ISPEC.EQ.3) THEN
C----- drive power to specified value
       P_SPEC = PSPEC / (RHO*VEL**3*RAD**2)
       DQ(K1) = PWAK + PVIS - P_SPEC
       DO J=1, II
         Q(K1,J) = PW_GAM(J) + PV_GAM(J)
       ENDDO
       Q(K1,K1) = PW_ADV + PV_ADV
       Q(K1,K2) = PW_ADW + PV_ADW
       Q(K1,K3) =          PV_DBE
C
       FRESMX = MAX( FRESMX , ABS(DQ(K1)) )
C
      ELSE IF(ISPEC.EQ.4) THEN
C----- fix advance ratio
       DQ(K1) = 0.
       DO J=1, II
         Q(K1,J) = 0.
       ENDDO
       Q(K1,K1) = 1.0
       Q(K1,K2) = 0.
       Q(K1,K3) = 0.
C
      ELSE IF(ISPEC.EQ.5) THEN
C----- drive power to value given by RPM
       P_SPEC = PSPEC / (RHO*VEL**3*RAD**2)
       P_SPEC_ADV = 0.0
       IF(LPWRVAR) THEN
         RPM  = VEL/(RAD*ADV*PI/30.)
         RPM_ADV  = -RPM/ADV
C
C----- fix 5/15/03 use linear interpolation for engine power/rpm line
ccc         CALL SEVLIN(RPM,PWRVAR,RPMVAR,NPWRVAR,PSPEC,PSPEC_RPM)
         PSPEC = SEVAL(RPM,PWRVAR,XPWRVAR,RPMVAR,NPWRVAR)
         PSPEC_RPM = DEVAL(RPM,PWRVAR,XPWRVAR,RPMVAR,NPWRVAR)
C
         PSPEC_ADV = PSPEC_RPM*RPM_ADV
C
         P_SPEC     = PSPEC    /(RHO*VEL**3*RAD**2)
         P_SPEC_ADV = PSPEC_ADV/(RHO*VEL**3*RAD**2)
       ENDIF
C
       DQ(K1) = PWAK + PVIS - P_SPEC
       DO J=1, II
         Q(K1,J) = PW_GAM(J) + PV_GAM(J)
       ENDDO
       Q(K1,K1) = PW_ADV + PV_ADV - P_SPEC_ADV
       Q(K1,K2) = PW_ADW + PV_ADW
       Q(K1,K3) =          PV_DBE
C
       FRESMX = MAX( FRESMX , ABS(DQ(K1)) )
C
      ENDIF
C
C---- Constraint conditions
      DQ(K3) = 0.
      DO J=1, K3
        Q(K3,J) = 0.
      ENDDO
      IF(ICON.EQ.1) Q(K3,K1) = 1.0      ! advance ratio(rpm) fixed
      IF(ICON.EQ.2) Q(K3,K3) = 1.0      ! blade pitch fixed
C
C---- solve linearized Newton system
      CALL GAUSS(IQ,K3,Q(1,1),DQ(1),1)
C
C
      RLX = 1.0
C---  Set initial iterations to underrelax
      IF(ITER .LE. 2) RLX = 0.2
C---- Apply limiters to the Newton updates based on physical properties
      DO I=1, II
        DGAM(I) = -DQ(I)
C
C---- limit CL changes near +- stall
        DCL = 2.0*DGAM(I) / (CH(I)*W)
        DCLMIN = MAX(1.5*DCLSTALL(I),ABS(CL(I)-CLMIN(I)))
        DCLMAX = MAX(1.5*DCLSTALL(I),ABS(CLMAX(I)-CL(I)))
C
        DCLLIM = MIN(0.5,DCLMIN,DCLMAX)
        DCLLIM = MAX(DCLLIM,0.01)
        IF(RLX*ABS(DCL) .GT. DCLLIM) THEN
          RLX = MIN(RLX,DCLLIM/ABS(DCL))
ccc      write(1,998) 'DCL lim i,rlx,cl,dcl ',i,rlx,cl(i),dcl,dcllim
ccc      write(1,998) 'clmax,clmin,dclstall ',i,clmax(i),clmin(i),
ccc     &              dclstall(i)
        ENDIF
 998    format(a,2x,i5,4(2x,F12.5))
C
C---- limit GAM changes that change sign
        IF(DGAM(I)*DGAMOLD(I) .LT. 0.0) THEN
          IF(ABS(DGAM(I)).GT.0.2*ABS(DGAMOLD(I))) THEN
            RLX = MIN(RLX,0.2)
cc        write(*,998) 'DGAM lim i,rlx,gam,dgam ',i,rlx,gam(i),
cc     &               dgam(i),dgamold(i)
          ENDIF
        ENDIF
C
      ENDDO
C
      DADV = -DQ(K1)
      DADW = -DQ(K2)
      DBET = -DQ(K3)
C
      IF(NITERA .EQ. 0) RLX = 0.0
C
C---- limit blade angle change to 0.05 radians  (~3 degrees)
      IF(RLX*DBET .GT. 0.05) RLX = MIN(RLX, 0.05/DBET)
      IF(RLX*DBET .LT. -.05) RLX = MIN(RLX,-0.05/DBET)
C
C---- limit advance ratio changes
c      IF(RLX*DADV .GT. 0.8*ADV) RLX = MIN(RLX,0.8*ADV/DADV)
c      IF(RLX*DADV .LT. -.5*ADV) RLX = MIN(RLX,-.5*ADV/DADV)
C
c      IF(RLX*DADW .GT. 0.8*ADW) RLX = MIN(RLX, 0.8*ADW/DADW)
c      IF(RLX*DADW .LT. -.5*ADW) RLX = MIN(RLX,-0.5*ADW/DADW)
C
      IF(RLX*DADV .GT. 0.5*ADV) RLX = MIN(RLX,0.5*ADV/DADV)
      IF(RLX*DADV .LT. -.3*ADV) RLX = MIN(RLX,-.3*ADV/DADV)
      IF(RLX*DADW .GT. 0.5*ADW) RLX = MIN(RLX, 0.5*ADW/DADW)
      IF(RLX*DADW .LT. -.3*ADW) RLX = MIN(RLX,-0.3*ADW/DADW)
C---- update circulation, blade angle arrays
      RMS = 0.
      GMX = 0.
      IMX = 0
      DO I=1, II
        GAM(I)   = GAM(I)   + RLX*DGAM(I)
        BETA(I)  = BETA(I)  + RLX*DBET
        BETA0(I) = BETA0(I) + RLX*DBET
C
        RMS = RMS + DGAM(I)**2 / (1.0 + 1.0/ADV**2)
        IF(ABS(DGAM(I)) .GE. ABS(GMX)) THEN
         GMX = DGAM(I)
         IMX = I
        ENDIF
        DGAMOLD(I) = DGAM(I)
      ENDDO
C
C---- update incremental blade angle
      DBETA = DBETA + RLX*DBET
C
C---- update advance ratios
      ADV = ADV + RLX*DADV
      ADW = ADW + RLX*DADW
C
      RMS  = SQRT(RMS/FLOAT(II))
C
C---- display iteration history
      WRITE(*,2100) ITER, GMX, IMX, RMS, 
     &              ADV, ADW, BETA(II)*180.0/PI, RLX
C
 2000 FORMAT(/' Iter     dGmax  @Imax    gGrms       Av        ',
     &        'Aw         Be       RLX')
 2100 FORMAT(1X,I3,3X,E10.3,2X,I3,2X,E10.3,2(2X,F8.4),2X,F8.3,2X,F8.4)
c
c Iter     dGmax    (I)    gGrms      Av        Aw         Be       RLX
cIIIXXXEEEEEEEEEEXXIIIXXEEEEEEEEEEXXFF.FFFFXXXFF.FFFFXXFFFF.FFFXXFFF.FFFF
c
C
C---- Smooth filter the GAM for low relaxation factors
        IF(RLX.LT.0.2) THEN
          WRITE(*,*) 'APITER filtering GAM'
          CALL FILTER(GAM,0.2*II,II)
        ENDIF
C
C---- test for convergence
      IF(RMS .LE. EPS) THEN
C----- final update of various quantities corresponding to converged solution
C
       IF(FREE) THEN
        IF(FAST) THEN
          CALL GRADMO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 1
        ELSEIF(.NOT.VRTX) THEN
          CALL HELICO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 2
        ELSEIF(VRTX) THEN
          CALL VRTXCO(IX,II, NBLDS, DUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
          IWTYP = 3
        ENDIF
       ENDIF
C
       CALL VCALC
       CALL TPQ(1)
C
       CONV = .TRUE.
       RETURN
      ENDIF
cc      IF(MOD(ITER,5).EQ.0) CALL APINIT
C
 1000 CONTINUE
C
      RETURN
      END ! APITER


      SUBROUTINE ADWCORR
C---- Calculate OMEG*R-VT correction factor to wake advance ratio equation
C
      INCLUDE 'XROTOR.INC'
      REAL ADVFACTOR(IX)
C
      DO I = 1, II
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,       SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
        ADVFACTOR(I) = XI(I)/ADV / (XI(I)/ADV - VT)
ccc        WRITE(18,*) XI(I), ADVFACTOR(I), XI(I)/ADV,VT
      END DO
C    
      RETURN
      END


      SUBROUTINE CSCALC(I,UTOT,WA,WT,
     &                  VT,VT_ADW,
     &                  VA,VA_ADW,
     &                  VD,VD_ADW,
     &                  CI,CI_ADV,CI_VT,
     &                  SI,             SI_VA,
     &                  W,  W_ADV, W_VT, W_VA,
     &                  PHI,P_ADV, P_VT, P_VA)
C
C---- Calculate velocity components at radial station I on real prop
C
      INCLUDE 'XROTOR.INC'
C
      VT = VIND(3,I)
      VT_ADW = VIND_ADW(3,I)
C     
      VA = VIND(1,I)
      VA_ADW = VIND_ADW(1,I)
C
C---- Include duct effect on freestream and induced axial velocity
      UDUCT     = 0.0
      VADUCT_VA = 1.0
      IF(DUCT) THEN
        UDUCT = URDUCT-1.0
        VADUCT_VA = 2.0*URDUCT
      ENDIF
C------ duct induced axial velocity
      VD    = VA * (VADUCT_VA - 1.0)
      VD_VA =      (VADUCT_VA - 1.0)
      VD_ADW = VD_VA*VA_ADW
C
C---- Freestream, body induced and added inflow velocities     
      UTOT = 1.0 + UDUCT + UBODY(I)
      CALL UVADD(XI(I),WA,WT)
C     
      CI     =  XI(I)/ADV - WT - VT
      CI_ADV = -XI(I)/ADV**2
      CI_VT  =                  -  1.0
C     
      SI     = UTOT + WA + VA  + VD
      SI_VA  =             1.0 + VD_VA
C
C---- Redefine VA to include duct induced velocity
ccc      VA     =  VA + VD
C     
      WSQ = CI*CI + SI*SI
      W = SQRT(WSQ)
      W_ADV = (CI*CI_ADV            )/W
      W_VT  = (CI*CI_VT             )/W
      W_VA  = (            SI*SI_VA )/W
C     
      PHI = ATAN2(SI,CI)
      P_ADV = (          - SI*CI_ADV)/WSQ
      P_VT  = (          - SI*CI_VT )/WSQ
      P_VA  = (CI*SI_VA             )/WSQ
C
cc      write(*,*) 'i,vt,va ',i,vt,va
      RETURN
      END ! CSCALC


      SUBROUTINE XWINIT
      INCLUDE 'XROTOR.INC'
C------------------------------------------------------------
C     Initial estimate for equivalent prop radial coordinate 
C     array (XW)
C------------------------------------------------------------
C
      UDUCT     = 0.0
      VADUCT_VA = 1.0
      IF(DUCT) THEN
        UDUCT = URDUCT-1.0
        VADUCT_VA = 2.0*URDUCT
      ENDIF

      XM = XW0
      DO I=1, II
        URAT = 1.0 + UDUCT + UBODY(I)
        DXW(I) = SQRT(XM**2 + 2.0*URAT*XI(I)*DXI(I)) - XM
        XP = XM + DXW(I)
        XW(I) = 0.5*(XP+XM)
C
        XW_ADV(I) = 0.
        XW_ADW(I) = 0.
        DO J=1, II
          XW_GAM(I,J) = 0.
        END DO
C
        VWAK(I) = VIND(3,I) * XI(I)/XW(I)
        VW_ADV(I) = 0.
        VW_ADW(I) = VIND_ADW(3,I) * XI(I)/XW(I)
        DO J=1, II
          VW_GAM(I,J) = VIND_GAM(3,I,J) * XI(I)/XW(I)
        END DO
C
        XM = XP
      END DO
C
      XWTIP = XM
C
      RETURN
      END ! XWINIT


      SUBROUTINE SETXW
      INCLUDE 'XROTOR.INC'
      REAL XWM_GAM(IX), Z_GAM(IX)
C---------------------------------------------------------------------
C     Calculates Xw (radial coordinate) and Vwak (Vtheta) for
C     the "equivalent prop" 
C     The radial stream function S xi dxi (Vax r dr) is used to 
C     define radial coordinate for the equivalent prop. The angular
C     momentum is preserved to define the equivalent prop Vwak (Vtheta)
C----------------------------------------------------------------------
C
      XWM = XW0
C
      DO J=1, II
        XWM_GAM(J) = 0.
      END DO
      XWM_ADV = 0.
      XWM_ADW = 0.
ccc      write(*,*) 'setxw adv,adw ',adv,adw
C
      DO 1000 I=1, II
        XDX = XI(I)*DXI(I)
C
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,             SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
C
C------ first guess for XWO
        DXWO = SQRT(XWM**2 + 2.0*UTOT*XDX) - XWM
        XWO = XWM + 0.5*DXWO
C
C------ Newton loop for XWO
        DO ITX=1, 30
C
          VW     =  VT * XI(I)/XWO
          VW_XWO = -VW/XWO
          VW_VT  =       XI(I)/XWO
C
C------ swirl velocity on equivalent prop
ccc************ not used 
          CW     = XWO/ADV - WT - VW
          CW_XWO = 1.0/ADV      - VW_XWO
C
          UTOTW = URDUCT
C------ axial velocity on equivalent prop (derived from swirl)
          VAW     =  VW*XWO /ADW
C------ no duct effect on freestream or axial induced velocity for equiv prop
          VAW_VW  =     XWO /ADW 
          VAW_XWO =  VW     /ADW + VAW_VW*VW_XWO 
C
          SW     = UTOTW   + WA + VAW
          SW_XWO =                VAW_XWO
C
          REZ     = SW*XWO*2.0*(XWO-XWM) - SI*XDX
          REZ_XWO = SW*2.0*(2.0*XWO-XWM) + SW_XWO*XWO*2.0*(XWO-XWM)
          DELXWO = -REZ/REZ_XWO
C
          RLX = 1.0
          IF(ABS(DELXWO) .GT. 0.2*(XWO-XWM)) 
     &             RLX = 0.2*(XWO-XWM)/ABS(DELXWO)
C
          XWO = XWO + RLX*DELXWO
          IF(ABS(DELXWO).LT.1.0E-6) GO TO 101
C
        END DO
        WRITE(*,990) 'SETXW: Xw convergence failed.  i, r/R, dXw :',
     &             I, XI(I), DELXWO
 990    FORMAT(A,I5,2(1X,F12.6))
C
  101   CONTINUE
C
        DXWO = 2.0*(XWO-XWM)
C
C------ Vw( xwo , Vt(Adw Gj) )
        VW     =  VT*XI(I)/XWO
        VW_XWO = -VW      /XWO
        VW_VT  =     XI(I)/XWO
C
C------ swirl velocity on equivalent prop
ccc************ not used 
        CW     =  XWO/ADV - WT - VW
        CW_XWO =  1.0/ADV      - VW_XWO
        CW_VT  =               - VW_VT
        CW_ADV = -XWO/ADV**2
C
        UTOTW = URDUCT
C------ axial velocity on equivalent prop (derived from swirl)
C------ no duct effect on freestream or axial induced velocity for equiv prop
        VAW     =  VW*XWO /ADW
        VAW_VW  =     XWO /ADW 
        VAW_XWO =  VW     /ADW
        VAW_ADW = -VAW    /ADW 
C
        SW     = UTOTW   + WA + VAW
        SW_XWO =                VAW_XWO + VAW_VW*VW_XWO 
        SW_VT  =                VAW_VW*VW_VT
        SW_ADW =                VAW_ADW
C
c        write(*,9999) 'setxw xi,xwvt,vw,sw ',xi(i),xwo,vt,vw,sw
c 9999   format(A,5F10.5)
C                        
C------ Res ( xwo , xwm , Sw(Adw Vt xw) , S(Adw Vt) )
CCC       REZ = SW*XWO*2.0*(XWO-XWM) - SI*XDX
        Z_XWO = SW*2.0*(2.0*XWO-XWM)
        Z_XWM =-SW*XWO*2.0
        Z_SW  =    XWO*2.0*(XWO-XWM)
        Z_SI  = -XDX
C
C------ Res ( xwo , xwm(Gj Adv Adw) , VT(Gj Adw) , VA(Gj Adw) , Adw )
        Z_XWO = Z_SW*SW_XWO               + Z_XWO
        Z_VT  = Z_SW*SW_VT 
        Z_VA  = Z_SI*SI_VA 
        Z_ADW = Z_SW*SW_ADW
C
C------ Res ( xwo , Adv , Adw , Gj )
        Z_ADV      = Z_XWM*XWM_ADV
        Z_ADW      = Z_XWM*XWM_ADW    + Z_VT*VT_ADW  
     &                                + Z_VA*VA_ADW + Z_ADW
        DO J=1, II
          Z_GAM(J) = Z_XWM*XWM_GAM(J) + Z_VT*VIND_GAM(3,I,J)
     &                                + Z_VA*VIND_GAM(1,I,J)
        END DO
C
C------ xwo( Adv , Adw , Gj )
        XW_ADV(I) = -Z_ADV/Z_XWO
        XW_ADW(I) = -Z_ADW/Z_XWO
        DO J=1, II
          XW_GAM(I,J) = -Z_GAM(J)/Z_XWO
        END DO
C
C------ Vw( xwo(Adv Adw Gj) , Vt(Adw Gj) )
        VWAK(I) = VW
C------ Vw( Adv Adw Gj )
        VW_ADV(I) = VW_XWO*XW_ADV(I)
        VW_ADW(I) = VW_XWO*XW_ADW(I) + VW_VT*VT_ADW
        DO J=1, II
          VW_GAM(I,J) = VW_XWO*XW_GAM(I,J) + VW_VT*VIND_GAM(3,I,J)
        END DO
C
C
        XW(I)  = XWO
C
C------ dxw( xwo(Adv Adw Gj) , xwm(Adv Adw Gj) )
        DXW(I) = 2.0*(XWO-XWM)
        DXW_ADV(I) = 2.0*(XW_ADV(I) - XWM_ADV)
        DXW_ADW(I) = 2.0*(XW_ADW(I) - XWM_ADW)
        DO J=1, II
          DXW_GAM(I,J) = 2.0*(XW_GAM(I,J) - XWM_GAM(J))
        END DO
C
C------ new  xwm(Adv Adw Gj)  for next loop pass
        XWM     = 2.0*XWO       - XWM
        XWM_ADV = 2.0*XW_ADV(I) - XWM_ADV
        XWM_ADW = 2.0*XW_ADW(I) - XWM_ADW
        DO J=1, II
          XWM_GAM(J) = 2.0*XW_GAM(I,J) - XWM_GAM(J)
        END DO
C
 1000 CONTINUE
C
      XWTIP = XWM
c      write(*,*) 'xwtip ',xwtip
c      do i=1,ii
c        write(20,*) 'xi,xw,vwak ',xi(i),xw(i),vwak(i)
c      end do
C
      RETURN
      END ! SETXW



      SUBROUTINE TPQ(ITYPE)
      INCLUDE 'XROTOR.INC'
C----------------------------------------------------------
C     Sets Thrust, Torque, Power, and their sensitivities 
C     wrt  beta, chord(i), Vtan(i), and lambda
C----------------------------------------------------------
C
      TINV = 0.
      PINV = 0.
C
      TWAK = 0.
      PWAK = 0.
C
      VAAavg = 0.
      VATavg = 0.
      TMOM  = 0.
      PMOM  = 0.
C
      TVIS = 0.
      PVIS = 0.
C
      TI_ADV = 0.
      PI_ADV = 0.
      TI_ADW = 0.
      PI_ADW = 0.
C
      TW_ADV = 0.
      PW_ADV = 0.
      TW_ADW = 0.
      PW_ADW = 0.
C
      TV_ADV = 0.
      PV_ADV = 0.
      TV_ADW = 0.
      PV_ADW = 0.
      TV_DBE = 0.
      PV_DBE = 0.
C
      DO I=1, II
        TI_GAM(I) = 0.
        PI_GAM(I) = 0.
        TW_GAM(I) = 0.
        PW_GAM(I) = 0.
        TV_GAM(I) = 0.
        PV_GAM(I) = 0.
      ENDDO
C
      COSR = COS(RAKE)
C
C---- go over radial stations, setting viscous thrust and power
      BLDS = FLOAT(NBLDS)
      DO 1000 I=1, II
        BDX = BLDS*DXI(I)
C
        XX = XI(I)/ADV
        XX_ADV = -XX/ADV
C
C------ set  W(Adv,Adw,Vt)  and  Phi(Adv,Adw,Vt)  sensitivities
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,             SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
C
          ALFA   = BETA(I) - PHI
          AL_DBE =  1.0
          AL_P   = -1.0
C
C
        IF(ITYPE.EQ.1) THEN
C------- analysis case:  fix local Beta (except for pitch change)
C
C------- set alfa(Gi,dBeta,Adv,Vt) sensitivites
         ALFA   = BETA(I) - PHI
         AL_GI  = 0.
         AL_DBE = 1.0
         AL_ADV = -P_ADV
         AL_VT  = -P_VT
         AL_VA  = -P_VA
C
C------- set CL(Gi,dBeta,Adv,Adw,Vt) sensitivites
         REY = CH(I)*ABS(W) * RHO*VEL*RAD/RMU
         CALL GETCLCDCM(I,ALFA,W,REY,
     &                  CL(I),CL_AL,CL_W,
     &                  CLMAX,CLMIN,DCLSTALL,STALL(I),
     &                  CD(I),CD_ALF,CD_W,CD_REY,
     &                  CM(I),CM_AL,CM_W)
         CL_GI  = CL_AL*AL_GI
         CL_DBE = CL_AL*AL_DBE
         CL_ADV = CL_AL*AL_ADV + CL_W*W_ADV
         CL_VT  = CL_AL*AL_VT  + CL_W*W_VT
         CL_VA  = CL_AL*AL_VA  + CL_W*W_VA
C
C------- set c(Gi,Adv,Vt) sensitivites  (chord is fixed)
         CH_GI  = 0.
         CH_ADV = 0.
         CH_VT  = 0.
         CH_VA  = 0.
C
        ELSE IF(ITYPE.EQ.2) THEN
C------- design case:  fix local CL and set chord based on circulation
C
C------- set alfa(Gi,dBeta,Adv,Adw,Vt) sensitivites
cc         write(*,*) 'tpq2 getalf i,cl,w ',i,cl(i),w
         CALL GETALF(I,CL(I),W,ALFA,AL_CL,AL_W,STALL(I))
         AL_GI  = 0.
         AL_DBE = 0.
         AL_ADV = AL_W*W_ADV
         AL_VT  = AL_W*W_VT
         AL_VA  = AL_W*W_VA
C
C------- set CL(Gi,dBeta,Adv,Adw,Vt) sensitivites
         CL_GI  = 0.
         CL_DBE = 0.
         CL_ADV = 0.
         CL_VT  = 0.
         CL_VA  = 0.
C
C------- set c(Gi,Adv,Adw,Vt) sensitivites
         CHNEW  = 2.0*GAM(I) / (W*CL(I))
C--- Check for chord going zero or negative and use nearby station data
C    for this iteration
         IF(CHNEW.LE.0.0) THEN
cc           write(*,*) 'TPQ negative chord @I = ',I,CHNEW
           IF(I.EQ.1) THEN
            CH(I)  = CH(I+1)
           ELSEIF(I.EQ.II) THEN
            CH(I)  = CH(I-1)
           ELSE
            CH(I)  = 0.5*(CH(I-1)+CH(I+1))
           ENDIF
            CH_GI  = 0.0
            CH_ADV = 0.0
            CH_VT  = 0.0
            CH_VA  = 0.0
         ELSE
          CH(I)  = 2.0*GAM(I) / (W*CL(I))
          CH_GI  = 2.0        / (W*CL(I))
          CH_ADV = (-CH(I)/W) * W_ADV
          CH_VT  = (-CH(I)/W) * W_VT
          CH_VA  = (-CH(I)/W) * W_VA
         ENDIF
C
         BETA(I) = ALFA + PHI
         BETA0(I) = BETA(I)
C
        ELSE IF(ITYPE.EQ.3) THEN
C------- design case:  fix local chord and set angles based on CL
C
C------- set CL(Gi,dBeta,Adv,Adw,Vt) sensitivites
         CL(I) = 2.0*GAM(I) / (W*CH(I))
         CL_GI = 2.0        / (W*CH(I))
         CL_DBE = 0.
         CL_ADV = (-CL(I)/W) * W_ADV
         CL_VT  = (-CL(I)/W) * W_VT
         CL_VA  = (-CL(I)/W) * W_VA
C
C------- set alfa(Gi,dBeta,Adv,Adw,Vt) sensitivites
cc         write(*,*) 'tpq3 getalf i,cl,w ',i,cl(i)
         CALL GETALF(I,CL(I),W,ALFA,AL_CL,AL_W,STALL(I))
         AL_GI  = AL_CL*CL_GI
         AL_DBE = AL_CL*CL_DBE
         AL_ADV = AL_CL*CL_ADV + AL_W*W_ADV
         AL_VT  = AL_CL*CL_VT  + AL_W*W_VT
         AL_VA  = AL_CL*CL_VA  + AL_W*W_VA
C
C------- set c(Gi,Adv,Adw,Vt) sensitivites
         CH_GI  = 0.
         CH_ADV = 0.
         CH_VT  = 0.
         CH_VA  = 0.
C
         BETA(I) = ALFA + PHI
         BETA0(I) = BETA(I)
C
        ENDIF
C
        RE(I) = CH(I)*ABS(W) * RHO*VEL*RAD/RMU
        RE_W  = CH(I)        * RHO*VEL*RAD/RMU
        RE_CH =       ABS(W) * RHO*VEL*RAD/RMU
C
C------ set Re(Gi,Adv,Adw,Vt) sensitivites
        RE_GI  = RE_CH*CH_GI
        RE_ADV = RE_CH*CH_ADV + RE_W*W_ADV
        RE_VT  = RE_CH*CH_VT  + RE_W*W_VT 
        RE_VA  = RE_CH*CH_VA  + RE_W*W_VA
C
C------ set CM and (not used at present) sensitivites
C------ set CD(Gi,dBeta,Adv,Adw,Vt) sensitivites
        CALL GETCLCDCM(I,ALFA,W,RE(I),
     &                 CL(I),CL_AL,CL_W,
     &                 CLMAX,CLMIN,DCLSTALL,STALL(I),
     &                 CD(I),CD_AL,CD_W,CD_RE,
     &                 CM(I),CM_AL,CM_W)
ccc        write(*,*) 'tpq alfa,cl,cd,cm ',i,alfa,cl(i),cd(i),cm(i)
        CD_GI  = CD_AL*AL_GI  + CD_RE*RE_GI
        CD_ADV = CD_AL*AL_ADV + CD_RE*RE_ADV + CD_W*W_ADV
        CD_VT  = CD_AL*AL_VT  + CD_RE*RE_VT  + CD_W*W_VT
        CD_VA  = CD_AL*AL_VA  + CD_RE*RE_VA  + CD_W*W_VA
        CD_DBE = CD_AL*AL_DBE 
C
C------ set total local efficiency
        EFF = (CL(I)*CI - CD(I)*SI)/(CD(I)*CI + CL(I)*SI) / XX
C---Correct for blade rake
        EFF = EFF*COSR
C
C------ set induced and profile local efficiencies
        EFFI = CI/(SI*XX)
C---Correct for blade rake
        EFFI = EFFI*COSR
C
        EFFP(I) = EFF / EFFI
C
        HWC    = 0.5*W*CH(I)
        HWC_W  = 0.5  *CH(I)
        HWC_CH = 0.5*W
C
C
C*******************************************************
C------ Viscous Thrust & Power contributions on real prop
cc      COSRV = COSR
        COSRV = 1.0
C
C------ dTv ( Cd , S , W , c ) sensitivites
        DTV    = -HWC   *CD(I)*SI*BDX * COSRV
C
        DTV_CD = -HWC         *SI*BDX * COSRV
        DTV_SI = -HWC   *CD(I)   *BDX * COSRV
        DTV_W  = -HWC_W *CD(I)*SI*BDX * COSRV
        DTV_CH = -HWC_CH*CD(I)*SI*BDX * COSRV
C
C------ set Tv(Gi,dBeta,Adv,Vt) sensitivites using chain rule
        DTV_GI  = DTV_CD*CD_GI  + DTV_CH*CH_GI
        DTV_DBE = DTV_CD*CD_DBE
        DTV_ADV = DTV_CD*CD_ADV + DTV_CH*CH_ADV
     &                          + DTV_W * W_ADV
        DTV_VT  = DTV_CD*CD_VT  + DTV_CH*CH_VT
     &                          + DTV_W * W_VT
        DTV_VA  = DTV_CD*CD_VA  + DTV_CH*CH_VA
     &          + DTV_SI*SI_VA  + DTV_W * W_VA
C
C------ accumulate viscous Thrust and sensitivities
        TVIS   = TVIS + DTV
        TV_ADV = TV_ADV + DTV_ADV
        TV_ADW = DTV_VT*VT_ADW + DTV_VA*VA_ADW
        TV_DBE = TV_DBE + DTV_DBE
C
        TV_GAM(I) = TV_GAM(I) + DTV_GI
        DO J=1, II
          TV_GAM(J) = TV_GAM(J) + DTV_VT*VIND_GAM(3,I,J)
     &                          + DTV_VA*VIND_GAM(1,I,J)
        ENDDO
C
C------ dPv( Cd , C , W , c ) 
        DPV    = HWC   *CD(I)*CI*BDX*XX
C
        DPV_CD = HWC         *CI*BDX*XX
        DPV_CI = HWC   *CD(I)   *BDX*XX
        DPV_W  = HWC_W *CD(I)*CI*BDX*XX
        DPV_CH = HWC_CH*CD(I)*CI*BDX*XX
C
C------ set Pv(Gi,dBeta,Adv,Vt) sensitivites using chain rule
        DPV_GI  = DPV_CD*CD_GI  + DPV_CH*CH_GI
        DPV_DBE = DPV_CD*CD_DBE
        DPV_ADV = DPV_CD*CD_ADV + DPV_CH*CH_ADV
     &          + DPV_CI*CI_ADV + DPV_W * W_ADV
     &          + HWC*CD(I)*CI*BDX*XX_ADV
        DPV_VT  = DPV_CD*CD_VT  + DPV_CH*CH_VT
     &          + DPV_CI*CI_VT  + DPV_W * W_VT
        DPV_VA  = DPV_CD*CD_VA  + DPV_CH*CH_VA
     &                          + DPV_W * W_VA
C
C------ accumulate viscous Power and sensitivities
        PVIS   = PVIS   + DPV
        PV_ADV = PV_ADV + DPV_ADV
        PV_ADW = DPV_VT*VT_ADW + DPV_VA*VA_ADW
        PV_DBE = PV_DBE + DPV_DBE
C
        PV_GAM(I) = PV_GAM(I) + DPV_GI
        DO J=1, II
          PV_GAM(J) = PV_GAM(J) + DPV_VT*VIND_GAM(3,I,J)
     &                          + DPV_VA*VIND_GAM(1,I,J)
        ENDDO
C
C
C*******************************************************
C------ Inviscid Thrust & Power contributions on real prop
cc      COSRI = COSR
        COSRI = 1.0
C
C------ dTi( Gi , C( Adv Vt ) )
        DTI = GAM(I)*CI*BDX * COSRI
C
        DTI_CI = GAM(I)*BDX * COSRI
        DTI_GI = CI*BDX * COSRI
C
C------ dTi( Adv , Vt(Adw Gj) )
        DTI_VT  = DTI_CI*CI_VT
        DTI_ADV = DTI_CI*CI_ADV
        DTI_ADW = DTI_VT*VT_ADW 
C
C------ accumulate inviscid Thrust and sensitivities
        TINV = TINV + DTI
        TI_ADV = TI_ADV + DTI_ADV
        TI_ADW = TI_ADW + DTI_ADW 
C------ Resolve dTi dependencies ( Vt ) to Gamma
        TI_GAM(I) = TI_GAM(I) + DTI_GI
        DO J=1, II
          TI_GAM(J) = TI_GAM(J) + DTI_VT*VIND_GAM(3,I,J)
        ENDDO
C
C------ dPi( S(Va) , Gi, Adv )
        DPI = GAM(I)*SI*BDX*XX
C
        DPI_SI = GAM(I)*BDX*XX
        DPI_GI = SI*BDX*XX
        DPI_XX = GAM(I)*SI*BDX
C
C------ dPi( Va(Gj Adw) , Adv , Adw , Gi )
        DPI_VA  = DPI_SI*SI_VA
        DPI_ADV = DPI_XX*XX_ADV
        DPI_ADW = DPI_VA*VA_ADW
C
C------ accumulate inviscid Power and sensitivities
        PINV = PINV + DPI
        PI_ADV = PI_ADV + DPI_ADV
        PI_ADW = PI_ADW + DPI_ADW 
C------ Resolve dPi dependencies ( Va ) to Gamma
        PI_GAM(I) = PI_GAM(I) + DPI_GI
        DO J=1, II
          PI_GAM(J) = PI_GAM(J) + DPI_VA*VIND_GAM(1,I,J)
        ENDDO
C
C*******************************************************


C*******************************************************
C------ Inviscid Thrust & Power contributions on equivalent prop
C       Assumes Omega and Gamma are same in real and equivalent prop
C
        VW = VWAK(I)
        UTOTW = URDUCT
        CALL UVADD(XI(I),WA,WT)
C
C------ Cw defined by same omega as real prop
        CW     =  XW(I)/ADV - WT  -  VW
        CW_ADV = -XW(I)/ADV**2
        CW_VW  =                  -  1.0
        CW_XW  =    1.0/ADV
C------ Sw( Adw , xw , Vw ) ;  xw, Vw( Gj , Adv , Adw )
        SW     = UTOTW + WA  +  VW*XW(I)/ADW
        SW_ADW =             -  VW*XW(I)/ADW**2
        SW_VW  =                   XW(I)/ADW
        SW_XW  =                VW      /ADW
C
C------ dTw( Gi , CW( Adv Vw ) , dxw( Gj, Adv, Adw) )
        DTW     = GAM(I)*CW*BLDS*DXW(I)
C
        DTW_GI  =        CW*BLDS*DXW(I)
        DTW_CW  = GAM(I)   *BLDS*DXW(I)
        DTW_DXW = GAM(I)*CW*BLDS
C------ dTw( Vt(Adw Gj) , Adv , Adw , Gi , dxw(Gj Adv Adw) )
        DTW_VW  = DTW_CW*CW_VW
        DTW_ADV = DTW_CW*CW_ADV    + DTW_VW * VW_ADV(I) 
     &                             + DTW_DXW*DXW_ADV(I)
        DTW_ADW = DTW_VW*VW_ADW(I) + DTW_DXW*DXW_ADW(I)
C
C------ accumulate Thrust and sensitivities
        TWAK   = TWAK   + DTW
        TW_ADV = TW_ADV + DTW_ADV
        TW_ADW = TW_ADW + DTW_ADW
C
C------ Resolve dTw dependencies ( Vt, Va, dxw ) to Gamma
        TW_GAM(I) = TW_GAM(I) + DTW_GI
        DO J=1, II
          TW_GAM(J) = TW_GAM(J) + DTW_VW *   VW_GAM(I,J)
     &                          + DTW_DXW*  DXW_GAM(I,J)
        ENDDO
C
C
C------ dPw( S(Va) , Gi , Adv )
        DPW     = GAM(I)*SI*BDX*XI(I)/ADV
C
        DPW_SI  = GAM(I)   *BDX*XI(I)/ADV
        DPW_GI  =        SI*BDX*XI(I)/ADV
        DPW_ADV = -DPW/ADV
C
C------ dPw( Adv , Adw , Va(Gj Adw) , Gi )
        DPW_VA  = DPW_SI*SI_VA
        DPW_ADW = DPW_VA*VA_ADW
C
C------ accumulate Power and sensitivities
        PWAK   = PWAK   + DPW
        PW_ADV = PW_ADV + DPW_ADV
        PW_ADW = PW_ADW + DPW_ADW
C
C------ Resolve dPw dependencies ( Va ) to Gamma
        PW_GAM(I) = PW_GAM(I) + DPW_GI
        DO J=1, II
          PW_GAM(J) = PW_GAM(J) + DPW_VA*VIND_GAM(1,I,J)
        ENDDO
C
c        write(*,1011) 'DTW DPW DTI DPI ',DTW,DPW,DTI,DPI,cw/sw,ci/si
c 1011   format(a,6F11.5)
C
C------ Save blade thrust and power contributions (per blade, per span)
        DTII(I) = DTI/BDX
        DPII(I) = DPI/BDX
        DTWI(I) = DTW/BDX
        DPWI(I) = DPW/BDX
        DTVI(I) = DTV/BDX
        DPVI(I) = DPV/BDX
C
ccc        write(21,*) XI(I),TWAK
C
C*******************************************************
C------ Inviscid Thrust & Power from momentum
C
C------ dTmom( S(Va) , Va )
        VTGM =  GAM(I)*BLDS/(4.0*PI*XI(I))
        VAGM =  VTGM*XI(I)/ADW
        DTM = 2.0*PI*XI(I)*DXI(I) * SI*(2.0*VA)
        TMOM = TMOM + DTM
ccc        write(20,*) XI(I),TMOM,VA,VD
C
C------ dPmom( S(Va) )
        DPM = DTM*(SI-VD)
        PMOM = PMOM + DPM
C
        VATavg = VATavg + DTW * (VA+VD)
        VAAavg = VAAavg + 2.0*PI*XI(I)*DXI(I) * (VA+VD)
C
 1000 CONTINUE
ccc        write(20,*) '&'
ccc        write(21,*) '&'
C
      TTOT = TWAK + TVIS
      PTOT = PWAK + PVIS
      QTOT = PTOT*ADV
ccc      write(*,*) 'TW,TI,TM ',TWAK,TINV,TMOM
c      write(*,*) 'TBLDdim ',TINV*RHO*VEL**2*RAD**2
c      write(*,*) 'TWAKdim ',TWAK*RHO*VEL**2*RAD**2
c      write(*,*) 'TTOTdim ',TTOT*RHO*VEL**2*RAD**2
c      write(*,*) 'TMOMdim ',TMOM*RHO*VEL**2*RAD**2
C
c      write(*,*) 'PBLDdim ',PINV*RHO*VEL**3*RAD**2
c      write(*,*) 'PWAKdim ',PWAK*RHO*VEL**3*RAD**2
c      write(*,*) 'PMOMdim ',PMOM*RHO*VEL**3*RAD**2
C
C---- disk area
      ADISK = PI*(1.0 - XI0**2)
      VAAavg = VAAavg/ADISK
      VATavg = VATavg/TWAK
c      write(*,*) '     VA Aavg ',VAAavg*VEL
c      write(*,*) '     VA Tavg ',VATavg*VEL
C
      TDIM = TWAK*RHO*VEL**2*RAD**2
      PDIM = PWAK*RHO*VEL**3*RAD**2
c      write(*,*) 'Vinduced from PWAK/TWAK ',PDIM/TDIM
C
      RETURN
      END ! TPQ



      SUBROUTINE VCALC
      INCLUDE 'XROTOR.INC'
C---------------------------------------------
C     Calculates cartesian induced velocities
C---------------------------------------------
      DO I=1, II
        VXSUM = 0.
        VYSUM = 0.
        VZSUM = 0.
        DO J=1, II
          VXSUM = VXSUM + VIND_GAM(1,I,J)*GAM(J)
          VYSUM = VYSUM + VIND_GAM(2,I,J)*GAM(J)
          VZSUM = VZSUM + VIND_GAM(3,I,J)*GAM(J)
        ENDDO
        VIND(1,I) = VXSUM
        VIND(2,I) = VYSUM
        VIND(3,I) = VZSUM
      ENDDO
C
      RETURN
      END ! VCALC



      SUBROUTINE GRADMO(IMAX,II, NBLDS, LDUCT, RAKE,
     &                XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
      DIMENSION XI(IMAX), XV(IMAX), GAM(IMAX)
      DIMENSION VIND_ADW(3,IMAX), VIND_GAM(3,IMAX,IMAX)
      LOGICAL LDUCT
C-----------------------------------------
C     Calculates "Graded Momentum" 
C     Gamma-swirl influence coefficients
C
C     Input:
C       IMAX         array dimension
C       II           number of radial points on blade (circulation stations)
C       NN           number of Fourier harmonics
C       NBLDS        number of blades
C       LDUCT        T for duct outer BC
C       XI(i)        radial coordinate array
C       GAM(i)       circulation array
C       ADW          wake advance ratio  V/wR
C
C     Output:
C
C     Output:
C       VIND_GAM(i,j)  sensitivity of velocity at i to circulation at j
C       VIND_ADW(i)    sensitivity of velocity at i to wake advance ratio
C
C        Where VIND_XXX(1,i,j) is the axial component
C              VIND_XXX(3,i,j) is the swirl component
C-----------------------------------------
      BLDS = FLOAT(NBLDS)
C
      PI = 4.0*ATAN(1.0)
C
      XI0   = XV(1)
      XITIP = XV(II+1)
C
      IF(LDUCT) THEN
C
C----- Circulation defines mean swirl at blade
C----- use simple mean swirl to get swirl at blade
       DO I=1, II
         DO J=1, II
           VIND_GAM(1,I,J) = 0.
           VIND_GAM(2,I,J) = 0.
           VIND_GAM(3,I,J) = 0.
         ENDDO
         VIND_GAM(3,I,I) =  BLDS/(4.0*PI*XI(I))
         VIND_ADW(3,I)   =  0.0
         VIND_ADW(2,I)   =  0.0
         VIND_GAM(1,I,I) =  VIND_GAM(3,I,I)*XI(I) /ADW
         VIND_ADW(1,I)   = -VIND_GAM(1,I,I)*GAM(I)/ADW
       ENDDO
C
      ELSE
C
C----- Circulation defines mean swirl at blade
C----- Free-tip treatment incorporates Prandtl's averaging factor F
       SFAC = SQRT(1.0 + 1.0/ADW**2)
       SF_ADW = 0.5/SFAC * (-2.0/ADW**3)
C
       DO 20 I=1, II
C
         DO J=1, II
           VIND_GAM(1,I,J) = 0.
           VIND_GAM(2,I,J) = 0.
           VIND_GAM(3,I,J) = 0.
         ENDDO
         VIND_ADW(1,I)   = 0.0
         VIND_ADW(2,I)   = 0.0
         VIND_ADW(3,I)   = 0.0
C
         ARG = MIN(20.0, 0.5*BLDS*(1.0 - XI(I)/XITIP)*SFAC)
         EK = EXP(-ARG)
         EK_ADW = -EK*0.5*BLDS*(1.0 - XI(I)/XITIP)*SF_ADW
         FK = SQRT(1.0 - EK*EK)
         FK_ADW = 0.5/FK * (-2.0*EK*EK_ADW)
         F = ATAN2(FK,EK)*2.0/PI
         F_ADW = (EK*FK_ADW - FK*EK_ADW)/(EK*EK + FK*FK) * 2.0/PI
C
         VIND_GAM(3,I,I) = BLDS/(4.0*PI*F*XI(I))
         VIND_ADW(3,I)   = BLDS/(4.0*PI*F*XI(I)) * GAM(I) * (-F_ADW/F)
         VIND_GAM(1,I,I) = VIND_GAM(3,I,I)*XI(I) /ADW
         VIND_ADW(1,I)   = VIND_ADW(3,I)  *XI(I) /ADW
     &                    -VIND_GAM(1,I,I)*GAM(I)/ADW
c
c--- Reverse VZ signs 
c         VIND_GAM(3,I,I) = -VIND_GAM(3,I,I)
c         VIND_ADW(3,I)   = -VIND_ADW(3,I)
ccc          VA_ADW = VIND_ADW(1,I) - VA/ADW
c
   20  CONTINUE
      ENDIF
C
      RETURN
      END ! GRADMO



      SUBROUTINE HELICO(IMAX,II, NBLDS, LDUCT, RAKE,
     &                  XI,XV,GAM, ADW,VIND_GAM,VIND_ADW) 
      DIMENSION XI(IMAX), XV(IMAX), GAM(IMAX)
      DIMENSION VIND_ADW(3,IMAX), VIND_GAM(3,IMAX,IMAX)
C
      LOGICAL LDUCT
C--------------------------------------------------------------------------
C     Calculates Swirl-Gamma influence coefficients by a mixed 
C     spectral/finite-difference method.
C
C     The modified Bessel equation for each perturbation potential
C     Fourier harmonic is solved by finite-differencing which gives 
C     a simple tri-diagonal system.
C
C     The equation for each harmonic is set up and forward-eliminated
C     in the same pass, which gives speed and storage efficiency at 
C     some expense of clarity.
C
C     Input:
C       IMAX         array dimension
C       II           number of radial points on blade (circulation stations)
C       NBLDS        number of blades
C       LDUCT        T for duct outer BC
C       XI(i)        r/R radial coordinate array
C       XV(i)        r/R vortex  leg   coordinate array
C       GAM(i)       circulation array
C       ADW          wake advance ratio  V/wR
C
C     Output:
C       VIND_GAM(i,j)  sensitivity of velocity at i to circulation at j
C       VIND_ADW(i)    sensitivity of velocity at i to wake advance ratio
C
C        Where VIND_XXX(1,i,j) is the axial component
C              VIND_XXX(3,i,j) is the swirl component
C--------------------------------------------------------------------------
      PARAMETER (IDIM = 150)
      DIMENSION X(0:IDIM), AINV(0:IDIM), CSAV(0:IDIM),
     &          AN_GAM(0:IDIM,0:IDIM), AN_ADW(0:IDIM)
      DIMENSION SYS(4,IDIM)
C
      IF(IDIM.LT.IMAX) STOP 'HELICO: Array overflow:  Increase IDIM.'
C
      PI = 4.0*ATAN(1.0)
C
C---- number of Fourier harmonics
      NN = 128
C
C
C---- set radial coordinate array for finite-difference solution
      DO I=1, II
        X(I) = XI(I)
      ENDDO
C
      XI0   = XV(1)
      XITIP = XV(II+1)
C
C---- radial coordinate array is also needed outside of blade
      X(0)    = 2.0*XI0   - XI(1)
      X(II+1) = 2.0*XITIP - XI(II)
C
      IF(LDUCT) THEN
        IIMAX = II+1
C
      ELSE
C------ position of outermost point (at "infinity")
        XINF = 4.0*XITIP
C
C------ set first few points beyond tip at mirror locations of those inside tip
        X(II+2) = 2.0*XITIP - XI(II-1)
        X(II+3) = 2.0*XITIP - XI(II-2)
        X(II+4) = 2.0*XITIP - XI(II-3)
C
C------ set remaining points with exponential stretching to outermost point
        XFAC = (X(II+4) - X(II+3)) / (X(II+3) - X(II+2))
        DX = (X(II+4) - X(II+3))*XFAC
        DO I = II+5, IDIM-1
          X(I) = X(I-1) + DX
          IF(X(I) .GE. XINF) GO TO 5
          DX = DX*XFAC
        ENDDO
        WRITE(*,*) 'HELICO: Local array too small. Increase IDIM.'
 5      CONTINUE
        IIMAX = I
      ENDIF
C
C
      DO I=1, II
        DO J=1, II
          VIND_GAM(1,I,J) = 0.
          VIND_GAM(2,I,J) = 0.
          VIND_GAM(3,I,J) = 0.
        ENDDO
        VIND_ADW(1,I) = 0.
        VIND_ADW(2,I) = 0.
        VIND_ADW(3,I) = 0.
      ENDDO
C
C==== Set up tridiagonal system
      ADWINV = 1.0/ADW**2
      QBSQ = 0.25*FLOAT(NBLDS)**2
C
      DO I = 1, IIMAX-1
        SYS(1,I) = (X(I  ) + X(I-1)) / (X(I  ) - X(I-1))
        SYS(3,I) = (X(I+1) + X(I  )) / (X(I+1) - X(I  ))
        SYS(2,I) = QBSQ*(1.0/X(I) + X(I)*ADWINV) * (X(I+1) - X(I-1))
        SYS(4,I) = QBSQ*(  -2.0*X(I)*ADWINV/ADW) * (X(I+1) - X(I-1))
      ENDDO
C
      I = IIMAX
      IF(LDUCT) THEN
        SYS(1,I) = -1.0
      ELSE
        SYS(1,I) =  1.0
      ENDIF
C
C==== Loop over all NN harmonics for n = 2,4,6,...
      DO 10 N = 2, NN, 2
        RN = FLOAT(N)
C
C------ set up and factor tridiagonal system for this n
C
C------ inner BC:  dAn/dx = 0
        AINV(0) =  1.0
        CSAV(0) = -1.0
C                                                2          2
C------ interior equations:  d[ x dAn/dx ]/dx - n K An  =  n K Gam
        DO I = 1, IIMAX-1
          B =   SYS(1,I)
          A = -(SYS(1,I) + SYS(2,I)*RN**2 + SYS(3,I))
          C =   SYS(3,I)
C
C-------- set 1 / (modified diagonal element)
          AINV(I) = 1.0 / (A - B*CSAV(I-1))
C
C-------- set normalized upper diagonal element for back substitution
          CSAV(I) = C*AINV(I)
        ENDDO
C
C------ outer BC:  dAn/dx = 0  (duct) ,  or   An = 0  (free tip)
        I = IIMAX
        B = SYS(1,I)
        A = 1.0
C
        AINV(I) = 1.0 / (A - B*CSAV(I-1))
C
C
C====== solve  An, dAn(i)/dGam(j) problems
C
C
C------ set righthand sides
        DO I = 0, IIMAX
          DO J = 0, II
            AN_GAM(I,J) = 0.
          ENDDO
        ENDDO
C
        DO I = 1, II
          AN_GAM(I,0) = SYS(2,I)*RN**2 * GAM(I)
          AN_GAM(I,I) = SYS(2,I)*RN**2
        ENDDO
C
C
C------ back-substitute RHSs
        DO I = 1, IIMAX
          IM = I-1
          B = SYS(1,I)
C
C-------- eliminate and normalize only up to nonzero elements
          JLAST = MIN(I,II)
          DO J = 0, JLAST
            AN_GAM(I,J) = (AN_GAM(I,J) - B*AN_GAM(IM,J))*AINV(I)
          ENDDO
        ENDDO
C
        DO I = IIMAX-1, 0, -1
          IP = I+1
          DO J = 0, II
            AN_GAM(I,J) = AN_GAM(I,J) - CSAV(I)*AN_GAM(IP,J)
          ENDDO
        ENDDO
C
C
C====== solve dAn(i)/dAdw problem
C
C------ set RHS
        AN_ADW(0) = 0.
        DO I = 1, IIMAX-1
          AN_ADW(I) = SYS(4,I)*RN**2 * (GAM(I) + AN_GAM(I,0))
        ENDDO
        AN_ADW(IIMAX) = 0.
C
C------ back-substitute RHS
        DO I = 1, IIMAX
          IM = I-1
          B = SYS(1,I)
          AN_ADW(I) = (AN_ADW(I) - B*AN_ADW(IM))*AINV(I)
        ENDDO
C
        DO I = IIMAX-1, 0, -1
          IP = I+1
          AN_ADW(I) = AN_ADW(I) - CSAV(I)*AN_ADW(IP)
        ENDDO
C
C
C------ sum potential harmonics to get Swirl-Gamma influence coefficients
        DO I=1, II
          DO J=1, II
            VIND_GAM(3,I,J) = VIND_GAM(3,I,J) + AN_GAM(I,J)
          ENDDO
C
          VIND_GAM(3,I,I) = VIND_GAM(3,I,I) + 1.0
          VIND_ADW(3,I)   = VIND_ADW(3,I)   + AN_ADW(I)
        ENDDO
C
 10   CONTINUE
C
C
C---- extrapolate the series to the next NN terms 
C-     assuming the known aymptotic behavior (An + Gam) ~ 1/n^2
C
      IF(.NOT.LDUCT) THEN
C
      FSUM = 0.
      DO N = NN+2, 4*NN, 2
        FSUM = FSUM + ( FLOAT(NN) / FLOAT(N) )**2
      ENDDO
C
      DO I=1, II
        DO J=1, II
          VIND_GAM(3,I,J) = VIND_GAM(3,I,J) + AN_GAM(I,J)*FSUM
        ENDDO
C
        VIND_GAM(3,I,I) = VIND_GAM(3,I,I) + 1.0      *FSUM
        VIND_ADW(3,I)   = VIND_ADW(3,I)   + AN_ADW(I)*FSUM
      ENDDO
C
      ENDIF
C
C---- Add on sawtooth self-influence term and scale properly
      DO I = 1, II
        BFAC = FLOAT(NBLDS)/(2.0*PI*X(I))
C
        VIND_GAM(3,I,I) = VIND_GAM(3,I,I) + 0.5
        DO J = 1, II
          VIND_GAM(3,I,J) = VIND_GAM(3,I,J) * BFAC
        ENDDO
        VIND_ADW(3,I) = VIND_ADW(3,I) * BFAC
C
      ENDDO
C
C---- Define other velocity components VX,VY from VZ
      DO I = 1, II
        VSUM = 0.0
        DO J = 1, II
         VIND_GAM(1,I,J) = VIND_GAM(3,I,J)*XI(I)/ADW
         VIND_GAM(2,I,J) = 0.0
         VSUM = VSUM + GAM(J)*VIND_GAM(3,I,J)
        ENDDO
cc        VIND_ADW(1,I) = VIND_ADW(3,I)*XI(I)/ADW 
        VIND_ADW(1,I) = VIND_ADW(3,I)*XI(I)/ADW - VSUM*XI(I)/ADW**2
        VIND_ADW(2,I) = 0.0
c
c--- Reverse VZ signs 
c        DO J = 1, II
c         VIND_GAM(3,I,J) = -VIND_GAM(3,I,J)
c        ENDDO
c        VIND_ADW(3,I) = -VIND_ADW(3,I)
C
      ENDDO
C
      RETURN
      END ! HELICO





      SUBROUTINE FILTER(Q,SMLEN,N)
C-----------------------------------------
C     Smooths array Q.
C     SMLEN is the number of points over
C     which information is smeared.
C-----------------------------------------
      IMPLICIT REAL (A-H,M,O-Z)
      DIMENSION Q(N)
C
      PARAMETER (NMAX=500)
      DIMENSION A(NMAX), B(NMAX), C(NMAX)
C
      IF(N.GT.NMAX) THEN
       WRITE(*,*) 'FILTER:  Array overflow.  No action taken'
       RETURN
      ENDIF
C
C---- set up and solve tridiagonal system for smoothed Q
C
      CON = SMLEN**2
      A(1) = 1.0
      C(1) = 0.
      DO 10 I=2, N-1
        B(I) =    -CON
        A(I) = 2.0*CON + 1.0
        C(I) =    -CON
   10 CONTINUE
      A(N) = 1.0
      B(N) = 0.
C
      CALL TRISOL(A,B,C,Q,N)
C
      RETURN
      END ! FILTER

