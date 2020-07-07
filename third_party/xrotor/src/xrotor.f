C***********************************************************************
C    Module:  xrotor.f
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
C
      PROGRAM XROTOR
C
C--- module statement for Windoze DVFortran
ccc   USE DFLIB
C
      INCLUDE 'XROTOR.INC'
      CHARACTER*7 COMAND
      CHARACTER*128 COMARG
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
C====================================================
C
C      Interactive Design and Analysis Program 
C          for Free-tip and Ducted Rotors
C
C      October 1992
C      Copyright Mark Drela
C      Versions 6.7-7.x 
C      Copyright Mark Drela, Harold Youngren
C
C====================================================
C
      VERSION = 7.55
C
C---- logical unit numbers
      LUREAD = 5    ! terminal read
      LUWRIT = 6    ! terminal write
      LUTEMP = 3    ! general-use disk I/O unit  (usually available)
      LUSAVE = 4    ! save file                  (usually open)
C
C
      WRITE(*,1000) VERSION
C
      CALL INIT
C
      FNAME = ' '
C--- Get command line args (if present)
      NARG = IARGC()
C
      IF(NARG.GT.0) CALL GETARG(1,FNAME)
      IF(FNAME(1:1) .NE. ' ') CALL LOAD(FNAME)
C
      FNAME = ' '
      IF(NARG.GT.1) CALL GETARG(2,FNAME)
      IF(FNAME(1:1) .NE. ' ') THEN
        NCASE = 0
        OPEN(LUTEMP,FILE=FNAME,STATUS='OLD',ERR=2)
        CALL GETCAS(LUTEMP,NPARX,NCASE,CASPAR)
        CLOSE(LUTEMP)
        IF(NCASE.GT.0) THEN
          KF = INDEX(FNAME,' ') - 1
          WRITE(*,*) 'Operating cases read from file  ',
     &                FNAME(1:KF),' ...'
          CALL SHOCAS(LUWRIT,NPARX,NCASE,CASPAR,RAD,NAME)
        ENDIF
 2      CONTINUE
      ENDIF
C
      WRITE(*,1100)
C
 900  CONTINUE
      CALL ASKC(' XROTOR^',COMAND,COMARG)
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
      GREEK = .TRUE.
      IF(COMAND.EQ.'    ') GO TO 900
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'QUIT') THEN
        IF(LPLOT) CALL PLCLOSE
        STOP
      ENDIF
      IF(COMAND.EQ.'AERO') CALL AERO
      IF(COMAND.EQ.'NAME') GO TO 10
      IF(COMAND.EQ.'ATMO') GO TO 20
      IF(COMAND.EQ.'VSOU') GO TO 22
      IF(COMAND.EQ.'DENS') GO TO 24
      IF(COMAND.EQ.'VISC') GO TO 26
      IF(COMAND.EQ.'DUCT') GO TO 60
      IF(COMAND.EQ.'VRAT') GO TO 65
C
      IF(COMAND.EQ.'II')   GO TO 75
      IF(COMAND.EQ.'INCR') GO TO 76
      IF(COMAND.EQ.'REIN') GO TO 78
C
      IF(COMAND.EQ.'NACE') CALL NACELL(COMARG)
      IF(COMAND.EQ.'ARBI') CALL ARBI
      IF(COMAND.EQ.'DESI') CALL DESI
      IF(COMAND.EQ.'MODI') CALL MODI
      IF(COMAND.EQ.'OPER') CALL OPER
      IF(COMAND.EQ.'BEND') CALL BEND
      IF(COMAND.EQ.'INTE') CALL INTE
      IF(COMAND.EQ.'INTF') CALL INTE2
      IF(COMAND.EQ.'SAVE') CALL SAVE(COMARG)
      IF(COMAND.EQ.'SAVO') CALL SAVEOLD(COMARG)
      IF(COMAND.EQ.'LOAD') CALL LOAD(COMARG)
      IF(COMAND.EQ.'WDEF') CALL WRTDEF
      IF(COMAND.EQ.'VPUT') CALL SAVVEL(COMARG)
      IF(COMAND.EQ.'VGET') CALL GETVEL(COMARG)
      IF(COMAND.EQ.'VCLR') GO TO 70
      IF(COMAND.EQ.'RESE') CALL INIT
      IF(COMAND.EQ.'JMAP') CALL JMAP
      IF(COMAND.EQ.'NOIS') CALL NOISE
      IF(COMAND.EQ.'HARD') GO TO 80
      IF(COMAND.EQ.'WIND') GO TO 85
      IF(COMAND.EQ.'PLOP') GO TO 90
      IF(COMAND.EQ.'DISP') GO TO 100
      IF(GREEK) WRITE(*,1050) COMAND
      GO TO 900
C
C--------------------------------------------------------------
 10   IF(COMARG.EQ.' ') THEN
        WRITE(*,*) 'Current name: ',NAME
        CALL ASKS('Enter new case name (32 characters max)^',COMARG)
      ENDIF
      IF(COMARG.NE.' ') NAME = COMARG
      GO TO 900
C
C--------------------------------------------------------------
 20   IF(NINPUT.GE.1) THEN
       ALT = RINPUT(1)
      ELSE
       CALL ASKR('flight altitude (km)^',ALT)
      ENDIF
      CALL ATMO(ALT,VSO,RHO,RMU)
      CALL FLOSHO(LUWRIT, VSO, RHO, RMU)
      GO TO 900
C
C--------------------------------------------------------------
 22   IF(NINPUT.GE.1) THEN
       VSO = RINPUT(1)
      ELSE
       CALL ASKR('fluid speed of sound (m/s)^',VSO)
      ENDIF
      ALT = 999.0
      GO TO 900
C
C--------------------------------------------------------------
 24   IF(NINPUT.GE.1) THEN
       RHO = RINPUT(1)
      ELSE
       CALL ASKR('fluid density (kg/m**3)^',RHO)
      ENDIF
      ALT = 999.0
      GO TO 900
C
C--------------------------------------------------------------
 26   IF(NINPUT.GE.1) THEN
       RMU = RINPUT(1)
      ELSE
       CALL ASKR('fluid dynamic viscosity (kg/m-s)^',RMU)
      ENDIF
      ALT = 999.0
      GO TO 900
C
C--------------------------------------------------------------
   60 DUCT = .NOT.DUCT
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
   65 IF(DUCT) THEN
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
C--------------------------------------------------------------
   70 NADD = 0
      GO TO 900
C
C--------------------------------------------------------------
 75   IF(NINPUT.GE.1) THEN
       IINEW = IINPUT(1)
      ELSE
       CALL ASKI('Enter # blade stations^',IINEW)
      ENDIF
      IF(LROTOR) CALL RESPACI(IINEW)
      II = IINEW
      GO TO 900
C
C--------------------------------------------------------------
 76   IF(NINPUT.GE.1) THEN
       INCR = IINPUT(1)
      ELSE
       CALL ASKI('Enter increment in output stations^',INCR)
      ENDIF
      GO TO 900
C
C--------------------------------------------------------------
 78   IF(LROTOR) CALL REINIT
      GO TO 900
C
C--------------------------------------------------------------
   80 IF(LPLOT) THEN
       CALL PLEND
       CALL REPLOT(IDEVRP)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C--------------------------------------------------------------
 85   WIND = .NOT.WIND
      IF(     WIND) WRITE(*,*) 'Windmill plotting mode set'
      IF(.NOT.WIND) WRITE(*,*) 'Propeller plotting mode set'
      GO TO 900
C
C--------------------------------------------------------------
   90 CALL OPLSET(IDEV,IDEVRP,IPSLU,
     &            SIZE,PAR,
     &            XMARG,YMARG,XPAGE,YPAGE,
     &            CSIZE,SCRNFR,LCURS,LLAND,LSLOPE)
      GO TO 900
C
C---------------------------------------------------------------------
  100 CALL OUTPUT(LUWRIT)
      GO TO 900
C
C.....................................................................
C
 1000 FORMAT(/' ========================='
     &       /'    XROTOR Version', F5.2             
     &       /' =========================')
 1050 FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
 1100 FORMAT(
     &  /'   QUIT   Exit program'
     &  /'  .AERO   Display or change airfoil characteristics'
     &  /'   NAME s Set or change case name'
     & //'   ATMO r Set fluid properties from standard atmosphere'
     &  /'   VSOU r Set/change fluid speed of sound'
     &  /'   DENS r Set/change fluid density'
     &  /'   VISC r Set/change fluid viscosity'
     & //'   NACE f Specify nacelle geometry file'
     &  /'   DUCT r Duct/free-tip option toggle'
     &  /'   VRAT r Change duct velocity ratio'
     & //'   ARBI   Input arbitrary rotor geometry'
     &  /'  .DESI   Design rotor geometry'
     &  /'  .MODI   Modify rotor geometry'
     &  /'  .OPER   Calculate off-design operating points'
     &  /'  .BEND   Calculate structural loads and deflections'
     &  /'  .NOIS   Calculate and plot acoustic signature'
     &  /'   JMAP   Calculate Cp vs J operating map'
     & //'   INTE   Interpolate geometry to specified radii'
     &  /'   SAVE f Save rotor to restart file'
     &  /'   SAVO f Save rotor to old style XROTOR restart file'
     &  /'   LOAD f Read rotor from restart file'
     &  /'   WDEF f Write current settings to  xrotor.def  file.'
     & //'   VPUT f Save  slipstream velocity profiles'
     &  /'   VGET f Read  slipstream velocity profiles'
     &  /'   VCLR   Clear slipstream velocity profiles'
     & //'   HARD   Hardcopy current plot'
     &  /'   WIND   Windmill/propeller plotting mode toggle'
     &  /'   PLOP   Plotting options'
     &  /'   DISP   Display current design point')
      END ! XROTOR


      SUBROUTINE INIT
      INCLUDE 'XROTOR.INC'
C--------------------------------------
C     Initializes everything
C--------------------------------------
C
      GREEK = .FALSE.
C
C---- Plotting flag
      IDEV = 1   ! X11 window only
c     IDEV = 2   ! B&W PostScript output file only (no color)
c     IDEV = 3   ! both X11 and B&W PostScript file
c     IDEV = 4   ! Color PostScript output file only 
c     IDEV = 5   ! both X11 and Color PostScript file 
C
C---- Re-plotting flag (for hardcopy)
      IDEVRP = 2   ! B&W PostScript
c     IDEVRP = 4   ! Color PostScript
C
C---- PostScript output logical unit and file specification
      IPSLU = 0  ! output to file  plot.ps   on LU 4    (default case)
c     IPSLU = ?  ! output to file  plot?.ps  on LU 10+?
C
C---- screen fraction taken up by plot window upon opening
      SCRNFR = 0.70
C
C---- Default plot size in inches
C-    (Default plot window is 11.0 x 8.5)
C-   (Must be smaller than XPAGE if objects are to fit on paper page)
      SIZE = 10.0

C---- plot-window dimensions in inches for plot blowup calculations
C-    currently,  11.0 x 8.5  default window is hard-wired in libPlt
      XPAGE = 11.0
      YPAGE = 8.5
C
C---- page margins in inches
      XMARG = 0.0
      YMARG = 0.0
C
      LEGEND = .FALSE.
C
      CALL PLINITIALIZE
C
C---- set up color spectrum
      NCOLOR = 64
      CALL COLORSPECTRUMHUES1(NCOLOR,'BCGYORM')
C
C---- plotting parameters
      LSLOPE = .FALSE.
C
C
C---- XROTOR defaults
      URDUCT = 1.0
C
      CALL SETDEF
      CALL GETDEF
C
      IF(DUCT) THEN
        WRITE(*,*) 'Aprop/Aexit initialized to 1.0'
        URDUCT = 1.0
      ENDIF
C
      XINF = 3.0        ! r/R at which BC at infinity is applied
      NN = 32           ! number of perturbation potential harmonics
      IINF = II + II/2  ! number of discrete potential harmonic stations
      CONV =  .FALSE.   ! operating point solution existence flag
      LPLOT = .FALSE.   ! active plot window status flag  (T = active)
      WIND = .FALSE.    ! Windmill flag for plotting Pc, Tc
      LSTRUC = .FALSE.  ! indicates if structural properties are available
C
      NAME = ' '
      SAVFIL = ' '
C
C---- acceleration due to gravity for scaling centrifugal blade tension (m/s^2)
      GEE = 9.81
C
C---- ADW factor (multiplies TINV/PINV in ADW calculation)
      ADWFCTR = 1.0
C
      IF(II  .GT.IX) STOP 'Array overflow.  IX too small'
      IF(IINF.GT.JX) STOP 'Array overflow.  JX too small'
C
C---- actual-rotor radius is always 1 (non-dimensionalized with itself)
      XITIP = 1.0
C
C---- default nacelle, wake perturbation velocities (non-existent)
      DO I=1, II
        UBODY(I) = 0.
      END DO
C
C---- no slipstream velocity profiles
      NADD = 0
C
C---- number of defined cases
      NCASE = 0
      KCASE = 0
C
C---- max number of iterations for design, analysis
      NITERD = 40
      NITERA = 40
C
C---- do not initialize rotor at each design cycle
      LDESINI = .FALSE.
C
C---- do initialize rotor at each design cycle
      LOPRINI = .TRUE.
C
C---- no engine load line to start
      LPWRVAR = .FALSE.
      NPWRVAR = 0
C
C---- no rotor yet
      LROTOR = .FALSE.
      DO I=1, IX
        IAERO(I) = 0
      END DO
C
C---- no x-y plot active yat
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = 0.
      XYFAC(2) = 0.
C
      RETURN
      END ! INIT



      SUBROUTINE SETDEF
      INCLUDE 'XROTOR.INC'
C
C---- hard-wired start-up defaults
cccIHI
      RAKE = 0.0
C
      VEL = 1.0
      ALT = 0.0
      CALL ATMO(ALT,VSO,RHO,RMU) ! sea level atmosphere parameters
CCC      RHO =  1.226      ! fluid density         kg/m**3
CCC      RMU =  1.78E-05   ! dynamic viscosity     kg/m-s
CCC      VSO =  340.0      ! speed of sound        m/s
C
C--- Default aero properties for section #1
      A0 = 0.           ! zero lift angle of attack   radians
      DCLDA =  6.28     ! lift curve slope            /radian
      CLMAX =   1.5     ! stall Cl
      CLMIN =  -0.5     ! negative stall Cl
      DCL_STALL =   0.1 ! CL increment from incipient to total stall
      DCLDA_STALL = 0.1 ! stalled lift curve slope    /radian
      CMCON = -0.1      ! section Cm  (for pitch-axis moments)
      CDMIN =  0.013    ! minimum Cd
      CLDMIN =  0.5     ! Cl at minimum Cd
      DCDCL2 =  0.004   ! d(Cd)/d(Cl**2)
      REREF =  200000.  ! Reynolds Number at which Cd values apply
      REXP =  -0.4      ! Exponent for Re scaling of Cd:  Cd ~ Re**exponent
      MCRIT =  0.8      ! Critical Mach number
C--- Install data into aero section #1
      NAERO = 1
      XISECT = 0.0
      CALL PUTAERO(NAERO,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      DO I=1, IX
        IAERO(I) = 1
      END DO
C
      XPITCH = 0.3     ! x/c location of pitch axis
C
      II =  30         ! number of radial stations
      INCR   = 2       ! radial station increment for terminal output
      IXSPAC = 2       ! r/R spacing flag
C
      VRTX =  .FALSE.  ! Vortex Wake (T)        / Graded Momentum(F) flag
      FAST =  .FALSE.  ! Graded Momentum(T)     / Potential Formulation(F) flag
      FREE =  .TRUE.   ! Self-deforming wake(T) / rigid wake(F) flag
      DUCT =  .FALSE.  ! Ducted (T)             / Free-tip (F)  flag
C
      TERSE = .FALSE.  ! terse-output flag
      LLAND = .TRUE.   ! landscape-mode plot flag
      LGRID = .TRUE.   ! grid plotting flag
C
      LVNORM = .TRUE.  ! flight speed used for normalization
C
      PAR = 0.6        ! plot aspect ratio
      CSIZE = 0.014    ! character size / plot-width
C
      RETURN
      END ! SETDEF


      SUBROUTINE GETDEF
      INCLUDE 'XROTOR.INC'
C
C---- try to read start-up defaults from xrotor.def file if possible
      LU = LUTEMP
      OPEN(LU,FILE='xrotor.def',STATUS='OLD',ERR=50)
C
C--- This data in the defaults file is no longer consistent with XROTOR 7+
C--- For now just use the non-aero data
      READ(LU,*,ERR=10) RHO, VSO, RMU
      READ(LU,*,ERR=10) DCLDA, A0
      READ(LU,*,ERR=10) CDMIN, DCDCL2, CLDMIN
      READ(LU,*,ERR=10) REREF, REXP
      READ(LU,*,ERR=10) CLMAX, CLMIN, DCL_STALL
      READ(LU,*,ERR=10) CMCON
      READ(LU,*,ERR=10) XPITCH
      READ(LU,*,ERR=10) II, INCR, IXSPAC
      READ(LU,*,ERR=10) FAST, FREE, DUCT
      READ(LU,*,ERR=10) TERSE, LLAND, LGRID
      READ(LU,*,ERR=10) LVNORM
      READ(LU,*,ERR=10) PAR, CSIZE
      CLOSE(LU)
C
      WRITE(*,*) ' '
      WRITE(*,*) 'Defaults read from file  xrotor.def'
      RETURN
C
 10   WRITE(*,*) ' '
      WRITE(*,*) 'READ error on file  xrotor.def'
      WRITE(*,*) 'Hard-wired defaults used'
      CLOSE(LU)
      CALL SETDEF
C
 50   WRITE(*,*) ' '
      WRITE(*,*) 'OPEN error on file  xrotor.def'
      WRITE(*,*) 'Hard-wired defaults used'
      CALL SETDEF
C
      RETURN
      END ! GETDEF



      SUBROUTINE WRTDEF
      INCLUDE 'XROTOR.INC'
      CHARACTER*1 ANS
C
      GREEK = .FALSE.
C
C---- try to open old xrotor.def file if possible
      LU = LUTEMP
      OPEN(LU,FILE='xrotor.def',STATUS='OLD',ERR=10)
C
      WRITE(*,*)
      WRITE(*,*) 'File  xrotor.def  exists.  Overwrite ?  Y'
      READ (*,1000) ANS
 1000 FORMAT(A)
      IF(INDEX('Nn',ANS) .EQ. 0) GO TO 15
C
      CLOSE(LU)
      WRITE(*,*) 'Default parameters not saved.'
      RETURN
C
 10   OPEN(LU,FILE='xrotor.def',STATUS='UNKNOWN',ERR=90)
 15   REWIND(LU)
C
C--- This data in the defaults file is no longer consistent with XROTOR 7+
C--- For now just use the first set of aero data
      A0          = AERODATA( 1,1)
      CLMAX       = AERODATA( 2,1)
      CLMIN       = AERODATA( 3,1)
      DCLDA       = AERODATA( 4,1)
      DCLDA_STALL = AERODATA( 5,1)
      DCL_STALL   = AERODATA( 6,1)
      CDMIN       = AERODATA( 7,1)
      CLDMIN      = AERODATA( 8,1)
      DCDCL2      = AERODATA( 9,1)
      CMCON       = AERODATA(10,1)
      REREF       = AERODATA(11,1)
      REXP        = AERODATA(12,1)
      MCRIT       = AERODATA(13,1)
      TOC         = AERODATA(14,1)
C
      WRITE(LU,*) RHO, VSO, RMU
      WRITE(LU,*) DCLDA, A0
      WRITE(LU,*) CDMIN, DCDCL2, CLDMIN
      WRITE(LU,*) REREF, REXP
      WRITE(LU,*) CLMAX, CLMIN, DCL_STALL
      WRITE(LU,*) CMCON
      WRITE(LU,*) XPITCH
      WRITE(LU,*) II, INCR, IXSPAC
      WRITE(LU,*) FAST, FREE, DUCT
      WRITE(LU,*) TERSE, LLAND, LGRID
      WRITE(LU,*) LVNORM
      WRITE(LU,*) PAR, CSIZE
      CLOSE(LU)
C
      WRITE(*,*) ' '
      WRITE(*,*) 'Defaults written to file  xrotor.def'
      RETURN
C
 90   WRITE(*,*) ' '
      WRITE(*,*) 'OPEN error.  Bad filename.'
      RETURN
      END ! WRTDEF



      SUBROUTINE ATMO(ALSPEC,VSOALT,RHOALT,RMUALT)
C---------------------------------------------------------
C     Returns speed of sound (VSO) in m/s, density (RHO)
C     in kg/m^3, and dynamic viscosity (RMU) in kg/m-s
C     of standard atmosphere at specified altitude ALSPEC
C     (in kilometers).  If ALSPEC=-1, water properties
C     at 15 Celsius are returned.
C
C     Reference:  "U.S. Standard Atmosphere", NOAA.
C---------------------------------------------------------
      LOGICAL FIRST
C
      PARAMETER ( N = 44 )
      REAL ALT(N), VSO(N), RHO(N), RMU(N)
C
      DATA FIRST / .TRUE. /
      DATA ALT
     &   / 0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,
     &    10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
     &    20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
     &    30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 
     &    40.0, 45.0, 60.0, 75.0 /
      DATA VSO
     & / 340.0,336.0,332.0,329.0,325.0,320.0,316.0,312.0,308.0,304.0,
     &   299.0,295.0,295.0,295.0,295.0,295.0,295.0,295.0,295.0,295.0,
     &   295.0,295.8,296.4,297.1,297.8,298.5,299.1,299.8,300.5,301.1,
     &   301.8,302.5,303.1,305.0,306.8,308.7,310.5,312.3,314.0,316.0,
     &   318.0,355.0,372.0,325.0 /
      DATA RHO
     & / 1.226,1.112,1.007,0.909,0.820,0.737,0.660,0.589,0.526,0.467,
     &   0.413,0.364,0.311,0.265,0.227,0.194,0.163,0.141,0.121,0.103,
     &   .0880,.0749,.0637,.0543,.0463,.0395,.0338,.0288,.0246,.0210,
     &   .0180,.0154,.0132,.0113,.0096,.0082,.0070,.0060,.0052,.0044,
     &   0.004,0.002,3.9E-4,8.0E-5 /
      DATA RMU
     & / 1.780,1.749,1.717,1.684,1.652,1.619,1.586,1.552,1.517,1.482,
     &   1.447,1.418,1.418,1.418,1.418,1.418,1.418,1.418,1.418,1.418,
     &   1.418,1.427,1.433,1.438,1.444,1.449,1.454,1.460,1.465,1.471,
     &   1.476,1.481,1.487,1.502,1.512,1.532,1.546,1.561,1.580,1.600,
     &   1.700,1.912,2.047,1.667 /
C
C---- special case: Water at STP
      IF(ALSPEC.EQ.-1.0) THEN
       VSOALT = 1500.
       RHOALT = 1000.
       RMUALT = 1.15E-3
       WRITE(*,*) '                              o        '
       WRITE(*,*) 'ATMO: You are underwater at 15  Celsius'
       RETURN
      ENDIF
C
C---- linearly interpolate quantities from tabulated values
      DO 10 I=2, N
        IF(ALSPEC.GT.ALT(I)) GO TO 10
C
         DALT = ALT(I) - ALT(I-1)
         DVSO = VSO(I) - VSO(I-1)
         DRHO = RHO(I) - RHO(I-1)
         DRMU = RMU(I) - RMU(I-1)
C
         ALFRAC = (ALSPEC - ALT(I-1)) / DALT
C
         VSOALT = VSO(I-1) + DVSO*ALFRAC
         RHOALT = RHO(I-1) + DRHO*ALFRAC
         RMUALT = RMU(I-1) + DRMU*ALFRAC
         RMUALT = RMUALT * 1.0E-5
C
         RETURN
   10 CONTINUE
C
C
      IF(ALSPEC.GT.ALT(N)) THEN
       WRITE(*,*) ' '
       WRITE(*,*) 'ATMO: You''re in low earth orbit.  Good luck.'
       VSOALT = VSO(N)
       RHOALT = RHO(N)
       RMUALT = RMU(N) * 1.0E-5
       RETURN
      ENDIF
C
c      IF(FIRST) THEN
c       DO 20 I=1, N
c         RHO(I) = ALOG(RHO(I))
c 20    CONTINUE
c       CALL SPLINE(VSO,VSOH,ALT,N)
c       CALL SPLIND(RHO,RHOH,ALT,N,999.0,0.0)
c       CALL SPLINE(RMU,RMUH,ALT,N)
c       FIRST = .FALSE.
c      ENDIF
cC
cC---- interpolate quantities from splines
c      VSOALT = SEVAL(ALSPEC,VSO,VSOH,ALT,N)
c      RHOALT = SEVAL(ALSPEC,RHO,RHOH,ALT,N)
c      RMUALT = SEVAL(ALSPEC,RMU,RMUH,ALT,N) * 1.0E-5
c      RHOALT = EXP(RHOALT)
cC
      RETURN
      END ! ATMO


      SUBROUTINE FLOSHO(LU, VSO, RHO, RMU)
      DATA R, GAM / 287.0, 1.4 /
      RNU = RMU/RHO
      P = RHO*VSO**2 / GAM
      T = P / (RHO*R)
      WRITE(LU,5000) VSO, RHO, RMU, RNU, P, T
 5000 FORMAT(/' Speed of sound (m/s):',F10.3
     &       /' Density   (kg/m^3)  :',F10.5
     &       /' Viscosity (kg/m-s)  :',E11.4
     &       /' Kin. Visc. (m^2/s)  :',E11.4
     &      //' Air pressure (Pa)   :',G13.5
     &       /' Air temperature (K) :',G12.4)
      RETURN
      END ! FLOSHO



      SUBROUTINE NACELL(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
      CHARACTER     FNTEMP*80
      PARAMETER (IZX=200)
      DIMENSION ABODY(IZX), ZBODY(IZX), RBODY(IZX), SOURCE(IZX)
C------------------------------------------------------------
C     Reads in specified nacelle cross-sectional area data
C     file and calculates associated disturbance velocity
C     field at all real prop radial stations.
C------------------------------------------------------------
C
      GREEK = .FALSE.
C
      LU = LUTEMP
C
      IF(RAD.EQ.0.0) THEN
       WRITE(*,*) 'Must define rotor geometry first'
       RETURN
      ENDIF
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ')
     & CALL ASKS('Enter nacelle area distribution filename^',FNAME)
C
      FNTEMP = FNAME
      CALL LC2UC(FNTEMP)
      IF(FNTEMP(1:5).EQ.'CONST') THEN
C---Constant Ubody specification, get value and fill arrays
        UCONS = 0.0
        VCONS = 0.0
        CALL ASKR('Enter constant Ubody^',UCONS)
cc      CALL ASKR('Enter constant Vbody^',VCONS)
        DO I=1, II
          UBODY(I) = UCONS
          VBODY(I) = VCONS
        END DO
        ZPROP = 0.0
        AWAKE = 0.0
        NZ = 2
        ZBODY(1) = 0.0
        ABODY(1) = 0.0
        ZBODY(2) = 1.0
        ABODY(2) = 0.0
        GO TO 20
C
      ELSE
C---Read in body definition and fill body velocity arrays
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=80)
      READ(LU,*,ERR=90) ZPROP, AWAKE
      IF(AWAKE .LT. 0.0) AWAKE = PI*AWAKE**2
      DO K=1, IZX
        READ(LU,*,END=11,ERR=90) ZBODY(K), ABODY(K)
        IF(ABODY(K) .LT. 0.0) ABODY(K) = PI*ABODY(K)**2
      END DO
   11 NZ = K-1
      CLOSE(LU)
C
      ENDIF
C
C---- non-dimensionalize with prop radius, and shift body to put prop at z=0
      ZPROP = ZPROP/RAD
      AWAKE = AWAKE/RAD**2
      DO K=1, NZ
        ZBODY(K) = ZBODY(K)/RAD    - ZPROP
        ABODY(K) = ABODY(K)/RAD**2
        RBODY(K) = SQRT( ABODY(K)/PI )
      END DO
C
C---HHY The SRCLIN routine attempts to calculate the strength of point sources
C       that satisfy velocity BC's on the body surface.  This implementation 
C       does not work for arbitrary body input Z,R point sets.  
C       The work-around is to set the point sources from body area deltas.
C
C---- Calculate body point sources
ccc      CALL SRCLIN(NZ,ZBODY,RBODY, SOURCE)
C---  FIX Set body point sources directly from area changes
      SOURCE(1) = 0.0
      DO I=2, NZ
        SOURCE(I) = ABODY(I) - ABODY(I-1)
      END DO
C
C---- evaluate source velocities at radial stations
      DO I=1, II
        CALL SRCVEL(NZ,ZBODY,SOURCE, 0.0,XI(I),UBODY(I),VBODY(I))
      ENDDO
C
C---- set wake core radius from far wake area
 20   XW0 = SQRT(AWAKE/PI)
C
      CALL XWINIT
C
      WRITE(*,1000) XW0, (I,UBODY(I),VBODY(I),XI(I), I=1,II)
      CONV = .FALSE.
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,SIZE,LPLOT,LLAND)
      CALL PLOTABS(0.5,0.75,-3)
      CALL NACPLT(NZ,ZBODY,RBODY,0.0,XW0, II,XI,UBODY)
C
      RETURN
C
   80 WRITE(*,*) 'Nacelle data file OPEN error'
      DO 85 I=1, II
        UBODY(I) = 0.
   85 CONTINUE
      WRITE(*,*) 'Nacelle perturbation velocities set to zero'
      RETURN
C
   90 WRITE(*,*) 'Nacelle data file READ error'
      DO 95 I=1, II
        UBODY(I) = 0.
   95 CONTINUE
      WRITE(*,*) 'Nacelle perturbation velocities set to zero'
      RETURN
C..............................................................
 1000 FORMAT(/' Far wake displacement body radius  r/R =',F10.4 /
     &       /' Radial distribution of disturbance velocities:'/
     &       /'   i   Uaxial/V   Uradial/V      r/R'
     &       / 50(1X, I3, 2F11.5, F10.4 /) )
C                10   -0.00123     0.874
      END ! NACELL



      SUBROUTINE ARBI
      INCLUDE 'XROTOR.INC'
C-----------------------------------------
C     Reads in arbitrary geometry input
C-----------------------------------------
      LOGICAL LCOR
C
      GREEK = .FALSE.
C
      CONV = .FALSE.
      CALL ASKI('number of blades   ^',NBLDS)
      CALL ASKR('flight speed (m/s) ^',VEL)
    2 CALL ASKR('tip radius (m)     ^',SGNRAD)
      RAD = ABS(SGNRAD)
      CALL ASKR('hub radius (m)     ^',ROOT)
      IF(RAD.LE.ROOT) GO TO 2
      XI0 = ROOT/RAD
      XW0 = ROOT/RAD
      CALL SETX
      CALL XWINIT
C
    5 CALL ASKI('number of radial stations^',NST)
      IF(NST.LT.3) WRITE(*,*) 'must have 3 or more stations'
      IF(NST.LT.3) GO TO 5
C
      WRITE(*,1010)
      DO 20 N=1, NST
  201   WRITE(*,1050) N
        READ (*,*,ERR=201) W1(N), W2(N), W3(N)
        IF(SGNRAD.LT.0.0) THEN
         W1(N) = W1(N)/RAD
         W2(N) = W2(N)/RAD
        ENDIF
        IF(W1(N).LT.XI0 .OR. W1(N).GT.XITIP) THEN
         WRITE(*,*) 'Radii must be between ',XI0,' and  1.0'
         GO TO 201
        ENDIF
        IF(N.GT.1 .AND. W1(N).LE.W1(N-1)) THEN
         WRITE(*,*) 'Radii must monotonically increase'
         GO TO 201
        ENDIF
   20 CONTINUE
C
      CALL ASKL('Any corrections?^',LCOR)
      IF(LCOR) THEN
   50  CALL ASKI('station number (0=quit)?^',N)
       IF(N.LE.0) GO TO 60
       IF(N.GT.NST) GO TO 50
  501  WRITE(*,1050) N
       READ (*,*,ERR=501) W1(N), W2(N), W3(N)
       GO TO 50
      ENDIF
C
   60 DO N=1, NST
        W3(N) = W3(N)*PI/180.
        W1(N) = TINVRT(W1(N))
      END DO
C
      CALL SPLINE(W2,W4,W1,NST)
      CALL SPLINE(W3,W5,W1,NST)
C
      DO I=1, II
        CH(I)   = SEVAL(T(I),W2,W4,W1,NST)
        BETA(I) = SEVAL(T(I),W3,W5,W1,NST)
        BETA0(I) = BETA(I)
        CH(I) = ABS(CH(I))                   ! negative chords are a no-no
        UBODY(I) = 0.
      END DO
C
      IF(NAME.EQ.' ') NAME = 'Arbitrary blade'
C
C---- estimate reasonable advance ratio to start iterative routines
      IS = II/2 + 1
C---HHY had to set A0 to 0.0 as A0 is now section property
      A0  = 0.0 
      ANG = BETA(IS) - A0
      ADV = XI(IS)*SIN(ANG)/COS(ANG)
      ADV = MAX(0.1,ADV)
      ADW = ADV
C
      CALL SETIAERO
      WRITE(*,1060)
      RETURN
C.......................................................................
C
 1010 FORMAT(/' For each station input:   r/R   chord/R   angle(deg)')
 1050 FORMAT( ' station ',I2,'   > ',$)
 1060 FORMAT(/' New rotor geometry created.'/)
      END ! ARBI


      SUBROUTINE REINIT
      INCLUDE 'XROTOR.INC'
      LOGICAL YES
C-----------------------------------------------
C     Re-initializes advance ratio and gammas
C-----------------------------------------------
C
C---- estimate reasonable advance ratio to start iterative routines
      IS = II/2 + 1
C---HHY had to set A0 to 0.0 as A0 is now section property
      A0  = 0.0 
      ANG = BETA(IS) - A0
C
      RPM  = VEL/(RAD*ADV*PI/30.)
C
      ADV0 = XI(IS)*SIN(ANG)/COS(ANG)
      RPM0 = VEL/(RAD*ADV0*PI/30.)

c      WRITE(*,*) 'Current    RPM ',RPM
c      WRITE(*,*) 'Initialize RPM ',RPM0
      CALL ASKR('Enter initialization RPM?^',RPM)

      ADV = VEL/(RPM*RAD*PI/30.)
      ADV = MAX(0.1,ADV)
      ADW = ADV
C
C---- Set the blade angle back to reference angle
      CALL ASKL('Restore blade angles to original?^',YES)
      IF(YES) THEN
        DO I = 1, II
          BETA0(I) = BETA(I)
        END DO
      ENDIF
C---- calculate current operating point
      CALL APER(4,2,.TRUE.)
      IF(CONV) CALL OUTPUT(LUWRIT)
C
      RETURN
      END ! REINIT


      SUBROUTINE INTE
      INCLUDE 'XROTOR.INC'
C-----------------------------------------------
C     Interpolates geometry to specified radii
C-----------------------------------------------
      LOGICAL LSAV
C
      GREEK = .FALSE.
C
      CALL SPLINE(CH,   T1,T,II)
      CALL SPLINE(BETA0,T2,T,II)
      CALL SPLINE(CL,   T3,T,II)
C
c      CALL SPLINE(SOUPL,T4,T,II)
c      CALL SPLINE(SINPL,T5,T,II)
c      CALL SPLINE(MOUPL,T6,T,II)
c      CALL SPLINE(MINPL,T7,T,II)
c      CALL SPLINE(TENVOL,T8,T,II)
C
      WRITE(*,*) 'Enter radius (r/R) for each station',
     & ' (enter 0 to terminate)...'
      DO 40 N=1, IX
C
  405   WRITE(*,1020) N
        READ (*,*,ERR=405) W0(N)
        IF(W0(N).EQ.0.0) GO TO 50
        IF(W0(N).LT.XI0 .OR. W0(N).GT.XITIP) THEN
         WRITE(*,*) 'Radii must be between ',XI0,' and', XITIP
         GO TO 405
        ENDIF
        IF(N.GT.1 .AND. W0(N).LE.W0(N-1)) THEN
         WRITE(*,*) 'Radii must monotonically increase'
         GO TO 405
        ENDIF
C
        XT = TINVRT(W0(N))
C
        W1(N) = SEVAL(XT,CH,   T1,T,II)*RAD
        W2(N) = SEVAL(XT,BETA0,T2,T,II)*180.0/PI
        W3(N) = SEVAL(XT,CL,   T3,T,II)
C
c        W4(N) = SEVAL(XT,SOUPL,T4,T,II)*RHO*VEL**2*RAD**2
c        W5(N) = SEVAL(XT,SINPL,T5,T,II)*RHO*VEL**2*RAD**2
c        W6(N) = SEVAL(XT,MOUPL,T6,T,II)*RHO*VEL**2*RAD**3
c        W7(N) = SEVAL(XT,MINPL,T7,T,II)*RHO*VEL**2*RAD**3
c        W8(N) = SEVAL(XT,TENVOL,T8,T,II)
   40 CONTINUE
C
   50 NR = N-1
C
      SFAC = 1.0
      CALL ASKR('Scale factor for R,C ?^',SFAC)
      CALL ASKL('Save to disk file ?^',LSAV)
      IF(LSAV) CALL OPFILE(LUSAVE,SAVFIL)
      WRITE(*,1100)
      WRITE(*,1200) SFAC
      IF(LSAV) WRITE(LUSAVE,1100)
      IF(LSAV) WRITE(LUSAVE,1200)
      DO N=1, NR
        RDIM = W0(N)*RAD*SFAC
        CDIM = W1(N)*SFAC
        WRITE(*,1300) W0(N),RDIM,CDIM,W2(N),W6(N),W7(N),W8(N)
        IF(LSAV) 
     &   WRITE(LUSAVE,1300) W0(N),RDIM,CDIM,W2(N),W6(N),W7(N),W8(N)
      END DO
      WRITE(*,1100)
      IF(LSAV) WRITE(LUSAVE,1100)
      IF(LSAV) CLOSE(LUSAVE)
C
      RETURN
C....................................................................
C
 1020 FORMAT(' ...station',I3,' :  ',$)
 1100 FORMAT(
     & /' =========================================================')
 1200 FORMAT('  Scale factor for radius,chord = ',F12.6,
     &/'   r/R   radius    chord    beta0   Moutplane  Minplane  T/blwt'
     &/'           (m)      (m)    (deg)     (N-m)      (N-m)')
C         0.250  0.42500  0.08240  64.231   1200.345   1345.476   5041.02
 1300 FORMAT(1X,
     &   F6.3,  F9.5,  F9.5,    F8.3,    F11.3,     F11.3,     F10.2)
      END ! INTE


      SUBROUTINE SETX
      INCLUDE 'XROTOR.INC'
C
C-------------------------------------------------------
C     Fills stretched radial coordinate array X (and XV)
C-------------------------------------------------------
C
      DT = 0.5*PI/FLOAT(II)
      XM = XI0
      XV(1) = XI0
      DO 10 I=1, II
        T(I) = DT*(FLOAT(I)-0.5)
        TP   = DT* FLOAT(I)
C
        IF(IXSPAC.EQ.2) THEN
C------- Usual sine stretching, adjusted for nonzero root radius
         XI(I) = SQRT(XITIP*SIN(T(I))**2 + (XI0*COS(T(I)))**2)
         XP    = SQRT(XITIP*SIN(TP  )**2 + (XI0*COS(TP  ))**2)
        ELSE
C------- Cosine stretching for more root resolution (also in TINVRT)
         XI(I) = 0.5*(1.0-COS(2.0*T(I)))*(XITIP-XI0) + XI0
         XP    = 0.5*(1.0-COS(2.0*TP  ))*(XITIP-XI0) + XI0
        ENDIF
C
        XI(I)  = (XP + XM)*0.5
        DXI(I) =  XP - XM
C
        XM = XP
        XV(I+1) = XP
   10 CONTINUE
      XV(II+1) = XITIP
C
      RETURN
      END ! SETX



      FUNCTION TINVRT(XX)
      INCLUDE 'XROTOR.INC'
C
C---------------------------------------------------------
C     Inverts X stretching function using Newton's method
C---------------------------------------------------------
C
      TT = ATAN2(XX,SQRT(XITIP**2-XX**2))
      DO 10 ITER=1, 40
        ST = SIN(TT)
        CT = COS(TT)
C
        IF(IXSPAC.EQ.2) THEN
C------- Usual sine stretching, adjusted for nonzero root radius
         REZ = (XITIP*ST)**2 + (XI0*CT)**2 - XX**2
         DRDT = 2.0*(XITIP**2-XI0**2)*ST*CT
        ELSE
C------- Cosine stretching for more root resolution (also in SETX)
         REZ = 0.5*(1.0-COS(2.0*TT))*(XITIP-XI0) + XI0 - XX
         DRDT = (XITIP-XI0)*SIN(2.0*TT)
        ENDIF
C
        TT = TT - REZ/DRDT
        IF(ABS(REZ).LE.1.E-5) GO TO 20
   10 CONTINUE
      WRITE(*,*) 'TINVRT: Convergence failed'
C
   20 TINVRT = TT
      RETURN
      END ! TINVRT




      SUBROUTINE OPFILE(LU,FNAME)
      CHARACTER*(*) FNAME
C
      CHARACTER*4 COMAND
      CHARACTER*128 COMARG,TMP
      CHARACTER*1 ANS, DUMMY
C
C---- get filename if it hasn't been already specified
      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter output filename^',FNAME)
C
C---- try to open file
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=50)
C
C---- file exists... ask how to proceed
      NF = INDEX(FNAME,' ') - 1
      TMP = 'File  '// FNAME(1:NF)//
     &      '  exists.  Overwrite / Append / New file ?^'
      CALL ASKC(TMP,COMAND,COMARG)
      ANS = COMAND(1:1)
C
C---- ask again if reply is invalid
      IF(INDEX('OoAaNn',ANS).EQ.0) THEN
        CALL ASKC(' O / A / N  ?^',COMAND,COMARG)
        ANS = COMAND(1:1)
C
        IF(INDEX('OoAaNn',ANS).EQ.0) THEN
C------- Still bad reply. Give up asking and just return
         WRITE(*,*) 'No action taken'
         RETURN
        ENDIF
      ENDIF
C
C---- at this point, file is open and reply is valid
      IF    (INDEX('Oo',ANS) .NE. 0) THEN
C------ go to beginning of file to overwrite
        REWIND(LU)
        GO TO 60
      ELSEIF(INDEX('Aa',ANS) .NE. 0) THEN
C------ go to end of file to append
        DO K=1, 12345678
          READ(LU,1000,END=60) DUMMY
 1000     FORMAT(A)
        ENDDO
      ELSE
C------ new file... get filename from command argument, or ask if not supplied
        FNAME = COMARG
        IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter output filename^',FNAME)
      ENDIF
C
C---- at this point, file FNAME is new or is to be overwritten
 50   OPEN(LU,FILE=FNAME,STATUS='UNKNOWN',ERR=90)
      REWIND(LU)
C
 60   RETURN
C
 90   WRITE(*,*) 'Bad filename.'
      RETURN
      END ! OPFILE
       

      SUBROUTINE OUTPUT(LU)
      INCLUDE 'XROTOR.INC'
      LOGICAL LHELI
      CHARACTER*1 SCHAR
C---------------------------------------------
C     Dumps operating state output to unit LU
C---------------------------------------------
C
      IADD = 1
      IF(LU.EQ.LUWRIT) IADD = INCR
C
      WRITE (LU,1000)
      IF(.NOT.CONV) WRITE(LU,2000)
C
      LHELI = .FALSE.
C
C---- dimensional thrust, power, torque, rpm
      TDIM = TTOT*RHO*VEL**2*RAD**2
      QDIM = QTOT*RHO*VEL**2*RAD**3
      PDIM = PTOT*RHO*VEL**3*RAD**2
C
      TVDIM = TVIS*RHO*VEL**2*RAD**2
      PVDIM = PVIS*RHO*VEL**3*RAD**2
C
      EFFTOT = TTOT/PTOT
      RPM = VEL/(RAD*ADV*PI/30.)
      DIA = 2.0*RAD
C
C---- Nacelle (or body) thrust is difference between thrust on 
C     equivalent prop and real prop
      TNACEL = (TWAK-TINV)*RHO*VEL**2*RAD**2
C
C---- blade solidity
      CALL SPLINE(CH,W1,XI,II)
      CH34 = SEVAL(0.75,CH,W1,XI,II)
      SIGMA = FLOAT(NBLDS)*CH34/PI
C
C---- standard coefficients based on forward speed
      TC = TDIM/(0.5*RHO*VEL**2 * PI*RAD**2)
      PC = PDIM/(0.5*RHO*VEL**3 * PI*RAD**2)
C
C---- standard coefficients based on rotational speed
      EN = RPM/60.0
      CT = TDIM/(RHO*EN**2*DIA**4)
      CP = PDIM/(RHO*EN**3*DIA**5)
C
C---- induced efficiency (including nacelle thrust effect)
      EFFIND = TWAK/PWAK
C
C---- ideal (actuator disk) efficiency
      TCLIM = MAX( -1.0 , TC )
      EIDEAL = 2.0 / (1.0 + SQRT(TCLIM + 1.0))
C
C---- define low advance ratio (helicopter?) related data
      IF(ADV.LT.0.1) THEN
       CALL SPLINE(CH,W1,XI,II)
       CTH  = CT/7.7516
       CPH  = CP/24.352
       CTOS = CTH / SIGMA
       FOM = 0.7979 * ABS(CT)**1.5 / CP
       LHELI = .TRUE.
      ENDIF
C
C
      IF(DUCT) THEN
       IF(IWTYP.EQ.1) WRITE(LU,1001) NAME
       IF(IWTYP.EQ.2) WRITE(LU,1002) NAME
       IF(IWTYP.EQ.3) WRITE(LU,1001) NAME
      ELSE
       IF(IWTYP.EQ.1) WRITE(LU,1011) NAME
       IF(IWTYP.EQ.2) WRITE(LU,1012) NAME
       IF(IWTYP.EQ.3) WRITE(LU,1013) NAME
      ENDIF
      IF(NADD.GT.1) THEN
       WRITE(LU,1021) ADW
      ELSE IF(DUCT) THEN
       WRITE(LU,1022) URDUCT, ADW
      ELSE
       WRITE(LU,1023) ADW
      ENDIF
      IF(ADW.LT.0.5*ADV) WRITE(LU,1024)
      WRITE(LU,1010) NBLDS,RAD,ADV,
     &               TDIM,PDIM,QDIM,
     &               EFFTOT,VEL,RPM,
     &               EFFIND,EIDEAL,TC,
     &               TNACEL,XI0*RAD,XW0*RAD,
     &               TVDIM,PVDIM,
     &               RHO,VSO,RMU
C
C---- low advance ratio (helicopter?) data
      IF(LHELI) THEN
       WRITE(LU,1116) SIGMA,CTOS,FOM
      ELSE
       WRITE(LU,1117) SIGMA
      ENDIF
C
C---- coefficients based on rotational speed
      WRITE(LU,1015) CT, CP, ADV*PI
C---- coefficients based on forward speed
      WRITE(LU,1016) TC, PC, ADV

cc      write(LU,1017) PVIS * ADV**3 * 2.0/PI,
cc     &               PWAK * ADV**3 * 2.0/PI 

C
      IF(TERSE) RETURN
C
C----- find maximum RE on blade
      REMAX = 0.0
      DO I=1, II
        REMAX = MAX(RE(I),REMAX)
      END DO
      REEXP = 1.0
      IF(REMAX.GE.1.0E6) THEN
        REEXP = 6.0
      ELSEIF(REMAX.GE.1.0E3) THEN
        REEXP = 3.0
      ENDIF
C
      IF(REEXP.EQ.1.0) THEN
        WRITE(LU,1020) 
       ELSE
        WRITE(LU,1120) IFIX(REEXP)
      ENDIF
C
      DO 10 I=1, II, IADD
C
C------ use equivalent prop to define local efficiency
        CALL UVADD(XI(I),WA,WT)
        VW  = VWAK(I)
        VAW = VW*XW(I)/ADW
C------ Freestream velocity component on equiv prop
        UTOTW = URDUCT
        CW = XI(I)/ADV - WT  -  VW
        SW = UTOTW     + WA  +  VAW
        EFFI = (CW/SW) * ADV/XW(I)
C
C------ use real prop to define Mach number
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,             SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
C
        MACH = W * VEL/VSO
C
        BDEG = BETA(I)*180./PI
        XRE = RE(I)/(10.0**REEXP)
C
        SCHAR = ' '
        IF(STALL(I)) SCHAR = 's'
C
        WRITE(LU,1030)
     &    I,XI(I),CH(I),BDEG,CL(I),SCHAR,CD(I),XRE,MACH,
     &    EFFI,EFFP(I),UBODY(I)
cc     &    ,rad*ch(i)*sin(beta(i))*39.36
   10 CONTINUE
cc      WRITE(LU,1000)
cc      WRITE(LU,*   ) ' '
C
      RETURN
C....................................................................
C
 1000 FORMAT(/1X,75('='))
 1001 FORMAT(' Ducted Graded Mom. Formulation Solution:  ', A32)
 1002 FORMAT(' Ducted Potential Formulation Solution:  ', A32)
 1011 FORMAT(' Free Tip Graded Mom. Formulation Solution:  ', A32)
 1012 FORMAT(' Free Tip Potential Formulation Solution:  ', A32)
 1013 FORMAT(' Free Tip Vortex Wake Formulation Solution:  ', A32)
 1021 FORMAT(' (External slipstream present)',19X,
     &           'Wake adv. ratio:',F11.5)
 1022 FORMAT(' Vdisk/Vslip:',F11.5,25X,
     &           'Wake adv. ratio:',F11.5)
 1023 FORMAT(50X,'Wake adv. ratio:',F11.5)
 1024 FORMAT(' Reverse far-slipstream velocity implied.',
     &   ' Interpret results carefully !')
 1010 FORMAT(' no. blades :',I3,  12X,'radius(m)  :',F9.4, 5X,
     &         'adv. ratio: ',F11.5,
     &      /' thrust(N)  :',G11.3,4X,'power(W)   :',G11.3,3X,
     &         'torque(N-m):',G11.3,
     &      /' Efficiency :',F8.4, 7X,'speed(m/s) :',F9.3, 5X,
     &         'rpm        :',F11.3,
     &      /' Eff induced:',F8.4, 7X,'Eff ideal  :',F9.4, 5X,
     &         'Tcoef      :',F11.4,
     &      /' Tnacel(N)  :',F11.4,4X,'hub rad.(m):',F9.4, 5X,
     &         'disp. rad. :',F10.4,
     &      /' Tvisc(N)   :',F11.4,4X,'Pvisc(W)   :',G11.3,
     &      /' rho(kg/m3) :',F10.5,5X,'Vsound(m/s):',F9.3, 5X,
     &         'mu(kg/m-s) :',E11.4
     &      /1X,75('-'))
 1015 FORMAT(12X,'    Ct:', F11.5, '     Cp:', F11.5, '    J:', F11.5)
 1016 FORMAT(12X,'    Tc:', F11.5, '     Pc:', F11.5, '  adv:', F11.5)
 1116 FORMAT('Helicopter: ',
     &       ' Sigma:', F11.5, '  CTh/s:', F11.5, '  FOM:', F11.5)
 1117 FORMAT(' Sigma:', F11.5)
 1017 FORMAT(' Cpv:', F11.5, '    Cpi:', F11.5 )
 1020 FORMAT(/'  i  r/R    c/R  beta(deg)',
     & '   CL      Cd    RE    Mach   effi  effp  na.u/U')
 1120 FORMAT(/'  i  r/R   c/R  beta(deg)',
     & '  CL     Cd    REx10^',I1,' Mach   effi  effp  na.u/U')
 1030 FORMAT(1X,I2,F6.3,F7.4,F7.2,F7.3,1X,A1,F7.4,1X,
     &       F6.2,1X,F6.3,1X,F6.3,F6.3,F8.3,f10.6)
 2000 FORMAT(/19X,'********** NOT CONVERGED **********'/)
      END ! OUTPUT



      SUBROUTINE RESPACI(IINEW)
      INCLUDE 'XROTOR.INC'
C
C--------------------------------------
C     Respaces points on current rotor
C--------------------------------------
C
      IF(IINEW.EQ.II) RETURN
C
      CONV = .FALSE.
C
C---- spline blade geometry to "old" radial locations
      DO I = 1, II
        W1(I) = XI(I)
        W2(I) = CH(I)
        W4(I) = BETA(I)
        W6(I) = UBODY(I)
      ENDDO
      CALL SPLINE(W2,W3,W1,II)
      CALL SPLINE(W4,W5,W1,II)
      CALL SPLINE(W6,W7,W1,II)
      IIX = II
      II  = IINEW
C
C---- set radial stations for built-in distribution scheme
      CALL SETX
      CALL XWINIT
C
C---- interpolate read-in geometry to generated radial stations
      DO I = 1, II
        CH(I)    = SEVAL(XI(I),W2,W3,W1,IIX)
        BETA(I)  = SEVAL(XI(I),W4,W5,W1,IIX)
        UBODY(I) = SEVAL(XI(I),W6,W7,W1,IIX)
        BETA0(I) = BETA(I)
      ENDDO
      IINF = II + II/2
C
C---- calculate current operating point
      CALL APER(4,2,.TRUE.)
      IF(CONV) CALL OUTPUT(LUWRIT)
C
C---- define design quantities for design of MIL prop with same parameters
      RADDES = RAD
      VELDES = VEL
      ADVDES = 0.
      RPMDES = VEL/(RAD*ADV) * 30.0/PI
      R0DES = XI0*RAD
      RWDES = XW0*RAD
      TDDES = TTOT*RHO*VEL**2*RAD**2
      PDDES = PTOT*RHO*VEL**3*RAD**2
      DEST = .FALSE.
      DESP = .TRUE.
      DO I=1, II
        CLDES(I) = CL(I)
      ENDDO
      CLDES0 = 0.
C
C---- rotor now exists
      LROTOR = .TRUE.
C
      RETURN
      END ! RESPACI


      SUBROUTINE CPROJ
      INCLUDE 'XROTOR.INC'
C---------------------------------------------------------
C     Calculates projected-chord radial integrals for
C     stability derivatives and whirl-flutter predictions.
C---------------------------------------------------------
C
      AX0 = 0.0
      AX1 = 0.0
      AT1 = 0.0
      AT2 = 0.0
C
      DO 10 I=1, II
C
        VT = VIND(3,I)
        VA = VIND(1,I)
C
        CI = XI(I)/ADV  -  VT
        SI = 1.0        +  VA
C
        W = SQRT(CI*CI + SI*SI)
C
        XX = XI(I)/ADV
        AX0 = AX0 + CH(I) * (1.0/W) * DXI(I)
        AX1 = AX1 + CH(I) * (1.0/W) * DXI(I) * XI(I)
        AT1 = AT1 + CH(I) * ( XX/W) * DXI(I) * XI(I)
        AT2 = AT2 + CH(I) * ( XX/W) * DXI(I) * XI(I)**2
C
 10   CONTINUE
C
      WRITE(*,1000) AX0, AX1, AT1, AT2
 1000 FORMAT(/1X,'/  c  V/W     dr =  Ax0 = ', F8.5,
     &       /1X,'|  c  V/W r   dr =  Ax1 = ', F8.5,
     &       /1X,'|  c wr/W r   dr =  At1 = ', F8.5,
     &       /1X,'/  c wr/W r^2 dr =  At2 = ', F8.5 )
      RETURN
      END ! CPROJ


      SUBROUTINE GETVEL(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
C--------------------------------------------------
C     Reads in incoming profile to be superimposed
C     on freestream and rotational components.
C--------------------------------------------------
C
      GREEK = .FALSE.
C
      LU = LUTEMP
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter input filename^',FNAME)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=95)
      DO 10 I=1, IX
        READ(LU,*,END=11,ERR=95) W1(I), W2(I), W3(I)
 10   CONTINUE
      I = IX+1
C
 11   IF(I.LT.3) GO TO 96
      IF(NADD.GE.2) THEN
       WRITE(*,*)
       WRITE(*,*) '*** Current slipstream profiles overwritten'
      ENDIF
      NADD = I-1
      CLOSE(LU)
C
      DO 20 I=1, NADD
        RADD(I) = W1(I)
        UADD(I) = W2(I)
        VADD(I) = W3(I)
 20   CONTINUE
C
      CALL ASKR('Enter axial velocity weight (0 ===> 1)^',UWT)
      CALL ASKR('Enter tang. velocity weight (0 , +/-1)^',VWT)
      DO 30 I=1, NADD
        UADD(I) = UWT*UADD(I)
        VADD(I) = VWT*VADD(I)
 30   CONTINUE
C
      CALL SPLINE(UADD,UADDR,RADD,NADD)
      CALL SPLINE(VADD,VADDR,RADD,NADD)
C
      WRITE(*,1200)
      WRITE(*,1250) (RADD(I), UADD(I), VADD(I), I=1, NADD)
      RETURN
C
 95   WRITE(*,*) 'File read error'
 96   WRITE(*,*) 'New wake velocities not read'
      
C....................................................
 1200 FORMAT(/' External slipstream velocity profiles:'
     &       /'      r (m)     Vaxi (m/s)  Vrot (m/s)')
CCC                 0.12341     0.12324    -0.08922
 1250 FORMAT(1X, 3F12.5)
      END
      

      SUBROUTINE SAVVEL(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
C--------------------------------------------------
C     Writes out far-downstream circumferentially-
C     averaged induced velocities for current 
C     propeller and operating state.
C--------------------------------------------------
      CHARACTER*1 ANS
C
      GREEK = .FALSE.
C
      LU = LUTEMP
C
      IF(RAD.EQ.0.0 .OR. .NOT.CONV) THEN
       WRITE(*,*) 'Must define operating point first.'
       RETURN
      ENDIF
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter output filename^',FNAME)
C     
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=5)
      WRITE(*,*)
      WRITE(*,*) 'Output file exists.  Overwrite?  Y'
      READ (*,1000) ANS
 1000 FORMAT(A)
      IF(INDEX('Nn',ANS).EQ.0) GO TO 6
C
      CLOSE(LU)
      WRITE(*,*) 'Velocities not saved.'
      RETURN
C
 5    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=90)
 6    REWIND(LU)
C
      BLDS = FLOAT(NBLDS)
      DO 10 I=1, II
C------ use circumferentially averaged induced velocity 
        VT = BLDS*GAM(I)/(4.0*PI*XI(I))
        VA = VT*XI(I)/ADW
C
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
        UTOT = 1.0 + UDUCT + UBODY(I)
        CALL UVADD(XI(I),WA,WT)
C
        CI = XI(I)/ADV - WT  -  VT
        SI = UTOT      + WA  +  VA + VD
C
        RDIM = XI(I)*RAD
        UDIM =  2.0*(SI - (UTOT      + WA))*VEL
        VDIM = -2.0*(CI - (XI(I)/ADV - WT))*VEL
C
        WRITE(LU,*) RDIM, UDIM, VDIM
 10   CONTINUE
      CLOSE(LU)
      RETURN
C
 90   WRITE(*,*) 'Bad filename.'
      WRITE(*,*) 'Velocities not saved.'
      RETURN
      END


      SUBROUTINE UVADD(XIW,WA,WT)
      INCLUDE 'XROTOR.INC'
C
      WA = 0.0
      WT = 0.0
C
      IF(NADD.LE.1) RETURN
C
      RDIM = XIW*RAD
      IF(RDIM.GE.RADD(1) .AND. RDIM.LE.RADD(NADD)) THEN
       WA = SEVAL(RDIM,UADD,UADDR,RADD,NADD) / VEL
       WT = SEVAL(RDIM,VADD,VADDR,RADD,NADD) / VEL
      ENDIF
C
      RETURN
      END


















