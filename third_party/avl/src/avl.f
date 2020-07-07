C***********************************************************************
C    Module:  avl.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
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

      PROGRAM AVL
C=======================================================================
C     3-D Vortex Lattice code.
C     See file avl_doc.txt for user guide.
C     See file version_notes.txt for most recent changes.
C=======================================================================
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      LOGICAL ERROR
C
      CHARACTER*4 COMAND
      CHARACTER*128 COMARG
      CHARACTER*80 FNNEW
C
      REAL    RINPUT(20)
      INTEGER IINPUT(20)
C
C      
      VERSION = 3.35
C
 1000 FORMAT(A)
C
      WRITE (*,1005) VERSION
 1005 FORMAT (
     &  /' ==================================================='
     &  /'  Athena Vortex Lattice  Program      Version ',F5.2
     &  /'  Copyright (C) 2002   Mark Drela, Harold Youngren'
     & //'  This software comes with ABSOLUTELY NO WARRANTY,' 
     &  /'    subject to the GNU General Public License.'
     & //'  Caveat computor'
     &  /' ===================================================')
C
C
      PI = 4.0*ATAN(1.0)
      DTR = PI/180.0
C
C---- logical units
      LUINP = 4   ! configuration file
      LURUN = 7   ! run case file
      LUMAS = 8   ! mass file
      LUPRM = 9   ! parameter file
      LUOUT = 19  ! output dump file
      LUSTD = 20  ! stability derivative dump file
      LUSYS = 22  ! dynamic system matrix dump file
C
C---- set basic defaults
      CALL DEFINI
      CALL MASINI
C
C---- initialize Xplot, and AVL plot stuff
      CALL PLINIT
C
C
C---- Read a new input geometry from input file
      CALL GETARG0(1,FILDEF)
C
      IF(FILDEF.NE.' ') THEN
       CALL INPUT(LUINP,FILDEF,ERROR)
C
C----- no valid geometry... skip reading run and mass files
       IF(ERROR) GO TO 100
C
C----- set up all parameters
       CALL PARSET
C
C----- process geometry to define strip and vortex data
       LPLTNEW = .TRUE.
       CALL ENCALC
C
C----- initialize state
       CALL VARINI
C
      ELSE
C----- no geometry... skip reading run and mass files
       GO TO 100
C
      ENDIF
C
C-------------------------------------------------------------------
C---- try to read mass file
      CALL GETARG0(3,FMSDEF)
      IF(FMSDEF.EQ.' ') THEN
       KDOT = INDEX(FILDEF,'.')
       IF(KDOT.EQ.0) THEN
        CALL STRIP(FILDEF,LENF)
        FMSDEF = FILDEF(1:LENF) // '.mass'
       ELSE
        FMSDEF = FILDEF(1:KDOT) // 'mass'
       ENDIF
      ENDIF
C
      CALL STRIP(FMSDEF,NMS)
      WRITE(*,*) 
      WRITE(*,*)
     & '---------------------------------------------------------------'
      WRITE(*,*) 'Trying to read file: ', FMSDEF(1:NMS), '  ...'
      CALL MASGET(LUMAS,FMSDEF,ERROR)
C
      IF(ERROR) THEN
       WRITE(*,*) 'Internal mass defaults used'
       CALL MASINI
C
      ELSE
       WRITE(*,*)
       WRITE(*,*) 'Mass distribution read ...'
       CALL MASSHO(6)
C
       CALL APPGET
       WRITE(*,*) 
     & '- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -'
       CALL APPSHO(6,RHO0)
C
      ENDIF
C
C-------------------------------------------------------------------
C---- try to read run case file
      CALL GETARG0(2,FRNDEF)
      IF(FRNDEF.EQ.' ') THEN
       KDOT = INDEX(FILDEF,'.')
       IF(KDOT.EQ.0) THEN
        CALL STRIP(FILDEF,LENF)
        FRNDEF = FILDEF(1:LENF) // '.run'
       ELSE
        FRNDEF = FILDEF(1:KDOT) // 'run'
       ENDIF
      ENDIF
C
      CALL STRIP(FRNDEF,NFR)
      WRITE(*,*)
      WRITE(*,*)
     & '---------------------------------------------------------------'
      WRITE(*,*) 'Trying to read file: ', FRNDEF(1:NFR), '  ...'
      CALL RUNGET(LURUN,FRNDEF,ERROR)
C
      IF(ERROR) THEN
       WRITE(*,*) 'Internal run case defaults used'
       CALL RUNINI
C
      ELSE
       WRITE(*,1025) (IR, RTITLE(IR), IR=1, NRUN)
 1025  FORMAT(//' Run cases read  ...',
     &        100(/1X,I4,': ',A))
C
      ENDIF
C
C-------------------------------------------------------------------
 100  CONTINUE
C
C---- set up plotting parameters for geometry (if any)
      CALL PLPARS
C
      WRITE(*,2000) 
 2000 FORMAT(
     &  /' =========================================================='
     &  /'   Quit    Exit program'
     & //'  .OPER    Compute operating-point run cases'
     &  /'  .MODE    Eigenvalue analysis of run cases'
     &  /'  .TIME    Time-domain calculations'
     & //'   LOAD f  Read configuration input file'
     &  /'   MASS f  Read mass distribution file'
     &  /'   CASE f  Read run case file'
     & //'   CINI    Clear and initialize run cases'
     &  /'   MSET i  Apply mass file data to stored run case(s)'
     & //'  .PLOP    Plotting options'
     &  /'   NAME s  Specify new configuration name')
C
C======================================================================
C---- start of menu loop
  500 CONTINUE
      CALL ASKC(' AVL^',COMAND,COMARG)
C
C---- extract command line numeric arguments
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 20
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 20
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
C===============================================
      IF(COMAND.EQ.'    ') THEN
       GO TO 500
C
C===============================================
      ELSEIF(COMAND.EQ.'?   ') THEN
       WRITE(*,2000)
C
C===============================================
      ELSEIF(COMAND.EQ.'QUIT' .OR.
     &       COMAND.EQ.'Q   '      ) THEN
       CALL PLCLOSE
       STOP
C
C===============================================
      ELSEIF(COMAND.EQ.'OPER') THEN
       CALL OPER
C
C===============================================
      ELSEIF(COMAND.EQ.'MODE') THEN
       CALL MODE
C
C===============================================
      ELSEIF(COMAND.EQ.'TIME') THEN
ccc       CALL TIME
C
C===============================================
      ELSE IF(COMAND.EQ.'LOAD') THEN
C----- Read a new input geometry from input file
       IF(COMARG.NE.' ') THEN
        FILDEF = COMARG
C
       ELSE
        CALL STRIP(FILDEF,LENF)
        LENF1 = MAX(LENF,1)
C
        WRITE(*,2010) FILDEF(1:LENF1)
 2010   FORMAT(' Enter input filename: ', A)
        READ (*,1000)  FNNEW
C
        IF(FNNEW.EQ.' ') THEN
         IF(LENF.EQ.0) GO TO 500
        ELSE
         FILDEF = FNNEW
        ENDIF
C
       ENDIF
C
       CALL INPUT(LUINP,FILDEF,ERROR)
       IF(ERROR) THEN
        WRITE(*,*) 
     &    '** File not processed. Current geometry may be corrupted.'
       GO TO 500
       ENDIF
C
       CALL PARSET
C
       IF(NRUN.EQ.0) THEN
        CALL RUNINI
       ELSE
        WRITE(*,*)
        WRITE(*,*) 'Existing run cases will be used.'
        WRITE(*,*) 'Issue CASE or CINI command if necessary.'
       ENDIF
C
C----- process geometry to define strip and vortex data
       LPLTNEW = .TRUE.
       CALL ENCALC
C
C----- initialize state
       CALL VARINI
C
       LAIC = .FALSE.
       LSRD = .FALSE.
       LVEL = .FALSE.
       LSOL = .FALSE.
       LSEN = .FALSE.
C
C----- set up plotting parameters for new geometry 
       CALL PLPARS
C
C===============================================
      ELSE IF(COMAND.EQ.'MASS') THEN
C----- Read a new mass distribution file
       IF(COMARG.NE.' ') THEN
        FMSDEF = COMARG
C
       ELSE
        CALL STRIP(FMSDEF,LENF)
        LENF1 = MAX(LENF,1)
C
        WRITE(*,3010) FMSDEF(1:LENF1)
 3010   FORMAT(' Enter mass filename: ', A)
        READ (*,1000)  FNNEW
C
        IF(FNNEW.EQ.' ') THEN
         IF(LENF.EQ.0) GO TO 500
        ELSE
         FMSDEF = FNNEW
        ENDIF
       ENDIF
C
       CALL STRIP(FMSDEF,NMS)
       CALL MASGET(LUMAS,FMSDEF,ERROR)
       IF(ERROR) THEN
       ELSE
        WRITE(*,*)
        WRITE(*,*) 'Mass distribution read ...'
        CALL MASSHO(6)
C
        CALL APPGET
        WRITE(*,*) 
     & '- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -'
        CALL APPSHO(6,RHO0)
C
        WRITE(*,*)
        WRITE(*,*) 
     &    'Use MSET to apply these mass,inertias to run cases'
ccc        CALL MASPUT(1,NRMAX)
       ENDIF
C
C===============================================
      ELSE IF(COMAND.EQ.'CASE') THEN
C----- Read a new run case file
       IF(COMARG.NE.' ') THEN
        FRNDEF = COMARG
C
       ELSE
        CALL STRIP(FRNDEF,LENF)
        LENF1 = MAX(LENF,1)
C
        WRITE(*,3020) FRNDEF(1:LENF1)
 3020   FORMAT(' Enter run case filename: ', A)
        READ (*,1000)  FNNEW
C
        IF(FNNEW.EQ.' ') THEN
         IF(LENF.EQ.0) GO TO 500
        ELSE
         FRNDEF = FNNEW
        ENDIF
       ENDIF
C
       CALL STRIP(FRNDEF,NFR)
       CALL RUNGET(LURUN,FRNDEF,ERROR)
       IF(ERROR) THEN
       ELSE
        WRITE(*,1025) (IR, RTITLE(IR), IR=1, NRUN)
       ENDIF
C
C----- initialize state
       CALL VARINI
C
       LSOL = .FALSE.
       LSEN = .FALSE.
C
C===============================================
      ELSE IF(COMAND.EQ.'CINI') THEN
       IF(LGEO) THEN
        CALL RUNINI
       ELSE
        WRITE(*,*) 'No configuration available.'
        NRUN = 0
       ENDIF
C
C===============================================
      ELSE IF(COMAND.EQ.'MSET') THEN
C----- set input mass,inertias
       IF(NINPUT.GE.1) THEN
        IR1 = IINPUT(1)
       ELSE
 60     WRITE(*,3060) 
 3060   FORMAT(/
     &     ' Enter index of target run case (0=all, -1=abort):  0')
        IR1 = 0
        CALL READI(1,IR1,ERROR)
        IF(ERROR) GO TO 60
       ENDIF
C
       IF(IR1.EQ.0) THEN
        IR1 = 1
        IR2 = NRUN
       ELSE
        IR2 = IR1
       ENDIF
C
       IF(IR1.LT.1 .OR. IR1.GT.NRUN) GO TO 500
C
       CALL MASPUT(IR1,IR2)
C
       LSOL = .FALSE.
       LSEN = .FALSE.
C
C===============================================
      ELSEIF(COMAND.EQ.'PLOP') THEN
       CALL OPLSET(IDEV,IDEVH,IPSLU,LSVMOV,
     &             SIZE,PLOTAR,
     &             XMARG,YMARG,XPAGE,YPAGE,
     &             CH,SCRNFRAC,LCURS,LCREV)
C
C===============================================
      ELSEIF(COMAND.EQ.'NAME') THEN
       IF(COMARG.EQ.' ') THEN
        CALL ASKS('Enter new name^',TITLE)
       ELSE
        TITLE = COMARG
       ENDIF
C
C===============================================
      ELSE
       WRITE(*,1050) COMAND
 1050  FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
C
      ENDIF
C
      GO TO 500
      END ! AVL


 
      SUBROUTINE PLINIT
C---- Initialize plotting variables
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      REAL ORG(3)
C
C---- Plotting flag
      IDEV = 1   ! X11 window only
c     IDEV = 2   ! B&W PostScript output file only (no color)
c     IDEV = 3   ! both X11 and B&W PostScript file
c     IDEV = 4   ! Color PostScript output file only 
c     IDEV = 5   ! both X11 and Color PostScript file 
C
C---- Re-plotting flag (for hardcopy)
c     IDEVH = 2    ! B&W PostScript
      IDEVH = 4    ! Color PostScript
C
C---- Movie-plotting flag
cc    IDEVH = 3    ! B&W PostScript
      IDEVM = 5   ! both X11 and Color PostScript file 
C
      LSVMOV = .FALSE.   ! no movie PS output yet
C
C---- PostScript output logical unit and file specification
ccc   IPSLU = -1  ! output to files plotNNN.ps on LU 80, with NNN = 001, 002, ...
      IPSLU = 0   ! output to file  plot.ps    on LU 80   (default case)
ccc   IPSLU = nnn ! output to file  plotNNN.ps on LU NNN
C
C---- screen fraction taken up by plot window upon opening
      SCRNFRAC = 0.70    ! Landscape
C     SCRNFRAC = -0.85   ! Portrait  specified if < 0
C
C---- Default plot size in inches
C-    (Default plot window is 11.0 x 8.5)
      SIZE = 9.0
C
C---- plot aspect ratio
      PLOTAR = 0.75
C
C---- character width/SIZE
      CH = 0.017
C
      CALL PLINITIALIZE
C
      NCOLORS = 0
C---- set up color spectrum
ccc      NCOLORS = 32
ccc      CALL COLORSPECTRUMHUES(NCOLORS,'RYGCBM')
C
C---- plot-window dimensions in inches for plot blowup calculations
C-    currently,  11.0 x 8.5  default window is hard-wired in libPlt
      XPAGE = 11.0
      YPAGE = 8.5
C
      XWIND = 11.0
      YWIND = 8.5
C
C---- page margins in inches
      XMARG = 0.0
      YMARG = 0.0
C
C---- bottom,left plot margin from edge
      PMARG = 0.15
C
      IF(IDEV.EQ.0) THEN 
        LPLOT = .FALSE.
      ENDIF
C

C---- set colors for run cases
      DO IR = 1, NRMAX
        IRCOLOR(IR) = MOD(IR-1,8) + 3
      ENDDO
C
C---- set vectors for little axes
      SLEN = 0.5
      HLEN = 0.5
C
      RHEAD = HLEN * 0.25
      NHEAD = NHAXIS
C
      ORG(1) = 0.
      ORG(2) = 0.
      ORG(3) = 0.
      DO IAX = 1, 3
        UAXDIR(1,IAX) = 0.
        UAXDIR(2,IAX) = 0.
        UAXDIR(3,IAX) = 0.
        UAXDIR(IAX,IAX) = 1.0
        CALL ARWSET(ORG,UAXDIR(1,IAX),SLEN,HLEN,RHEAD,NHEAD,
     &                  UAXARW(1,1,1,IAX),NLINAX)
      ENDDO
C
C---- initial phase, eigenvector scale, slo-mo scale (for mode plots)
      EPHASE = 0.0
      EIGENF = 1.0
      SLOMOF = 1.0
      TMOFAC = 1.0

      RETURN
      END ! PLINIT



      SUBROUTINE PLPARS
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      IMARKSURF = 0
      DO N = 1, NSURF
        LPLTSURF(N) = .TRUE. 
      END DO
      DO N = 1, NBODY
        LPLTBODY(N) = .TRUE. 
      END DO
C
C---- Scaling factors for velocity and pressure
      CPFAC = MIN(0.4*CREF,0.1*BREF)  / CREF
      ENFAC = MIN(0.3*CREF,0.06*BREF) / CREF
      HNFAC = MIN(CREF,0.5*BREF)      / CREF
C
C---- initialize observer position angles and perspective 1/distance
      AZIMOB = -45.0
      ELEVOB =  20.0
      TILTOB =   0.
      ROBINV = 0.
C
C---- slo-mo factor
      SLOMOF = 1.0
C
C---- eigenmode animation integration time step
      DTIMED = 0.025
C
C---- movie-dump frame time step
      DTMOVIE = 0.05
C
C---- max length of movie
      TMOVIE = 10.0
C
C...Flags 
      LABEL_BODY = .FALSE.
      LABEL_SURF = .FALSE.
      LABEL_STRP = .FALSE.
      LABEL_VRTX = .FALSE.
      LWAKEPLT   = .FALSE.
      LHINGEPLT  = .FALSE.
      LLOADPLT   = .FALSE.
      LCNTLPTS   = .FALSE.
      LNRMLPLT   = .FALSE.
      LAXESPLT   = .TRUE.
      LRREFPLT   = .TRUE.
      LCLPERPLT  = .TRUE.
      LDWASHPLT  = .TRUE.
      LLABSURF   = .FALSE.
      LCAMBER    = .FALSE.
      LCHORDLINE = .TRUE.
      LBOUNDLEG  = .TRUE.
C
C---- Initially assume nothing hidden
      LHID = .TRUE.
C
C---- Initially assume no reverse color output
      LCREV = .FALSE.
C
C---- flags to plot parameter values above eigenmode map
      DO IP = 1, IPTOT
        LPPAR(IP) = .FALSE.
      ENDDO

      LPPAR(IPALFA) = .TRUE.
      LPPAR(IPBETA) = .TRUE.
c      LPPAR(IPROTX) = .TRUE.
c      LPPAR(IPROTY) = .TRUE.
c      LPPAR(IPROTZ) = .TRUE.
      LPPAR(IPCL  ) = .TRUE.
      LPPAR(IPCD0 ) = .TRUE.

      LPPAR(IPPHI ) = .TRUE.
c      LPPAR(IPTHE ) = .TRUE.
c      LPPAR(IPPSI ) = .TRUE.

c      LPPAR(IPMACH) = .TRUE.
      LPPAR(IPVEE ) = .TRUE.
      LPPAR(IPRHO ) = .TRUE.
c      LPPAR(IPGEE ) = .TRUE.

      LPPAR(IPRAD ) = .TRUE.
c      LPPAR(IPFAC ) = .TRUE.

      LPPAR(IPXCG ) = .TRUE.
c      LPPAR(IPYCG ) = .TRUE.
      LPPAR(IPZCG ) = .TRUE.

      LPPAR(IPMASS) = .TRUE.
c      LPPAR(IPIXX ) = .TRUE.
c      LPPAR(IPIYY ) = .TRUE.
c      LPPAR(IPIZZ ) = .TRUE.
c      LPPAR(IPIXY ) = .TRUE.
c      LPPAR(IPIYZ ) = .TRUE.
c      LPPAR(IPIZX ) = .TRUE.

c      LPPAR(IPCLA ) = .TRUE.
c      LPPAR(IPCLU ) = .TRUE.
c      LPPAR(IPCMA ) = .TRUE.
c      LPPAR(IPCMU ) = .TRUE.

      RETURN
      END ! PLPARS



      SUBROUTINE DEFINI
      INCLUDE 'AVL.INC'
C
C---- flag for forces in standard NASA stability axes (as in Etkin)
      LNASA_SA  = .TRUE.
C
C---- flag for rotations defined in stability axes or body axes
      LSA_RATES = .TRUE.
C
      LPTOT   = .TRUE.
      LPSURF  = .FALSE.
      LPSTRP  = .FALSE.
      LPELE   = .FALSE.
      LPHINGE = .FALSE.
      LPDERIV = .FALSE.
C
      LGEO  = .FALSE.
      LENC  = .FALSE.
C
      LAIC  = .FALSE.
      LSRD  = .FALSE.
      LVEL  = .FALSE.
      LSOL  = .FALSE.
      LSEN  = .FALSE.
C
      LVISC    = .TRUE.
      LBFORCE  = .TRUE.
      LTRFORCE = .TRUE.
C
      LMWAIT = .FALSE.
C
      MATSYM = 0
      NITMAX = 20
C
      SAXFR = 0.25  ! x/c location of spanwise axis for Vperp definition
C
      VRCORE = 0.25   ! vortex core radius / vortex span
      SRCORE = 0.75   ! source core radius / body radius
C
C---- dafault basic units
      UNITL = 1.
      UNITM = 1.
      UNITT = 1.
      UNCHL = 'Lunit'
      UNCHM = 'Munit'
      UNCHT = 'Tunit'
      NUL = 5
      NUM = 5
      NUT = 5
C
C---- set corresponding derived units
      CALL UNITSET
C
C---- default air density and grav. accel.
      RHO0 = 1.0
      GEE0 = 1.0
C
C---- no eigenvalue reference data yet
      FEVDEF = ' '
      DO IR = 1, NRMAX
        NEIGENDAT(IR) = 0
      ENDDO
C
C---- no run cases defined yet
      NRUN = 0
      IRUN = 1
C
C---- number of valid time levels stored
      NTLEV = 0
C
C---- default time step, and number of time steps to take
      DELTAT = 0.0
      NTSTEPS = 0
C
      RETURN
      END ! DEFINI



      SUBROUTINE PARSET
      INCLUDE 'AVL.INC'
C
C---- variable names
      VARNAM(IVALFA) = 'alpha '
      VARNAM(IVBETA) = 'beta  '
      VARNAM(IVROTX) = 'pb/2V '
      VARNAM(IVROTY) = 'qc/2V '
      VARNAM(IVROTZ) = 'rb/2V '
C
C---- variable selection keys
      VARKEY(IVALFA) = 'A lpha'
      VARKEY(IVBETA) = 'B eta'
      VARKEY(IVROTX) = 'R oll  rate'
      VARKEY(IVROTY) = 'P itch rate'
      VARKEY(IVROTZ) = 'Y aw   rate'
C
C---- constraint names
CCC                     123456789012
      CONNAM(ICALFA) = 'alpha '
      CONNAM(ICBETA) = 'beta  '
      CONNAM(ICROTX) = 'pb/2V '
      CONNAM(ICROTY) = 'qc/2V '
      CONNAM(ICROTZ) = 'rb/2V '
      CONNAM(ICCL  ) = 'CL    '
      CONNAM(ICCY  ) = 'CY    '
      CONNAM(ICMOMX) = 'Cl roll mom'
      CONNAM(ICMOMY) = 'Cm pitchmom'
      CONNAM(ICMOMZ) = 'Cn yaw  mom'
C
C---- constraint selection keys
      CONKEY(ICALFA) = 'A '
      CONKEY(ICBETA) = 'B '
      CONKEY(ICROTX) = 'R '
      CONKEY(ICROTY) = 'P '
      CONKEY(ICROTZ) = 'Y '
      CONKEY(ICCL  ) = 'C '
      CONKEY(ICCY  ) = 'S '
      CONKEY(ICMOMX) = 'RM'
      CONKEY(ICMOMY) = 'PM'
      CONKEY(ICMOMZ) = 'YM'
C
C------------------------------------------------------------------------
      IZERO = ICHAR('0')
C
C---- add control variables, direct constraints
      DO N = 1, NCONTROL
        ITEN = N/10
        IONE = N - 10*ITEN
C
C------ assign slots in variable ond constraint lists
        IV = IVTOT + N
        IC = ICTOT + N
        VARNAM(IV) = DNAME(N)
        CONNAM(IC) = DNAME(N)
        IF(ITEN.EQ.0) THEN
         VARKEY(IV) = 'D' // CHAR(IZERO+IONE) // ' '
     &             // ' ' // DNAME(N)(1:8)
         CONKEY(IC) = 'D' // CHAR(IZERO+IONE)
        ELSE
         VARKEY(IV) = 'D' // CHAR(IZERO+ITEN) // CHAR(IZERO+IONE)
     &             // ' ' // DNAME(N)(1:8)
         CONKEY(IC) = 'D' // CHAR(IZERO+ITEN) // CHAR(IZERO+IONE)
        ENDIF
C
        LCONDEF(N) = .TRUE.
      ENDDO
C
C---- default design-variable flags, names
      DO N = 1, NDESIGN
        LDESDEF(N) = .TRUE.
      ENDDO
C
C---- total number of variables, constraints
      NVTOT = IVTOT + NCONTROL
      NCTOT = ICTOT + NCONTROL
C
C---- run-case parameter names
      PARNAM(IPALFA) = 'alpha    '
      PARNAM(IPBETA) = 'beta     '
      PARNAM(IPROTX) = 'pb/2V    '
      PARNAM(IPROTY) = 'qc/2V    '
      PARNAM(IPROTZ) = 'rb/2V    '
      PARNAM(IPCL )  = 'CL       '
      PARNAM(IPCD0)  = 'CDo      '
      PARNAM(IPPHI)  = 'bank     '
      PARNAM(IPTHE)  = 'elevation'
      PARNAM(IPPSI)  = 'heading  '
      PARNAM(IPMACH) = 'Mach     '
      PARNAM(IPVEE)  = 'velocity '
      PARNAM(IPRHO)  = 'density  '
      PARNAM(IPGEE)  = 'grav.acc.'
      PARNAM(IPRAD)  = 'turn_rad.'
      PARNAM(IPFAC)  = 'load_fac.'
      PARNAM(IPXCG)  = 'X_cg     '
      PARNAM(IPYCG)  = 'Y_cg     '
      PARNAM(IPZCG)  = 'Z_cg     '
      PARNAM(IPMASS) = 'mass     '
      PARNAM(IPIXX)  = 'Ixx      '
      PARNAM(IPIYY)  = 'Iyy      '
      PARNAM(IPIZZ)  = 'Izz      '
      PARNAM(IPIXY)  = 'Ixy      '
      PARNAM(IPIYZ)  = 'Iyz      '
      PARNAM(IPIZX)  = 'Izx      '
      PARNAM(IPCLA)  = 'visc CL_a'
      PARNAM(IPCLU)  = 'visc CL_u'
      PARNAM(IPCMA)  = 'visc CM_a'
      PARNAM(IPCMU)  = 'visc CM_u'
C
C---- total number of parameters
      NPTOT = IPTOT
C
C---- set default parameter unit names
      CALL PARNSET
C
      RETURN
      END ! PARSET



      SUBROUTINE PARNSET
      INCLUDE 'AVL.INC'
C
C---- set parameter unit name
      DO IP = 1, IPTOT
        PARUNCH(IP) = ' '
      ENDDO
C
      PARUNCH(IPALFA) = 'deg'
      PARUNCH(IPBETA) = 'deg'
      PARUNCH(IPPHI)  = 'deg'
      PARUNCH(IPTHE)  = 'deg'
      PARUNCH(IPPSI)  = 'deg'
      PARUNCH(IPVEE)  = UNCHV
      PARUNCH(IPRHO)  = UNCHD
      PARUNCH(IPGEE)  = UNCHA
      PARUNCH(IPRAD)  = UNCHL

C---- bug  21 Feb 13   MD
c      PARUNCH(IPXCG)  = UNCHL
c      PARUNCH(IPYCG)  = UNCHL
c      PARUNCH(IPZCG)  = UNCHL
      PARUNCH(IPXCG)  = 'Lunit'
      PARUNCH(IPYCG)  = 'Lunit'
      PARUNCH(IPZCG)  = 'Lunit'

      PARUNCH(IPMASS) = UNCHM
      PARUNCH(IPIXX)  = UNCHI
      PARUNCH(IPIYY)  = UNCHI
      PARUNCH(IPIZZ)  = UNCHI
      PARUNCH(IPIXY)  = UNCHI
      PARUNCH(IPIYZ)  = UNCHI
      PARUNCH(IPIZX)  = UNCHI
C
      RETURN
      END ! PARNSET



      SUBROUTINE VARINI
      INCLUDE 'AVL.INC'
C
C---- initialize state
      ALFA = 0.
      BETA = 0.
      WROT(1) = 0.
      WROT(2) = 0.
      WROT(3) = 0.
C
      DO N = 1, NCONTROL
        DELCON(N) = 0.0
      ENDDO
C
      DO N = 1, NDESIGN
        DELDES(N) = 0.0
      ENDDO
      LSOL = .FALSE.
C
      RETURN
      END ! VARINI



      SUBROUTINE RUNINI
      INCLUDE 'AVL.INC'
C
      WRITE(*,*)
      WRITE(*,*) 'Initializing run cases...'
C
C---- go over all run cases
      DO IR = 1, NRMAX
C------ index of default constraint for each variable
        ICON(IVALFA,IR) = ICALFA
        ICON(IVBETA,IR) = ICBETA
        ICON(IVROTX,IR) = ICROTX
        ICON(IVROTY,IR) = ICROTY
        ICON(IVROTZ,IR) = ICROTZ
C
C------ default constraint values
        DO IC = 1, ICTOT
          CONVAL(IC,IR) = 0.
        ENDDO
C
C------ default run case titles
        RTITLE(IR) = ' -unnamed- '
C
C------ default dimensional run case parameters
        DO IP = 1, NPTOT
          PARVAL(IP,IR) = 0.
        ENDDO
        PARVAL(IPGEE,IR) = GEE0
        PARVAL(IPRHO,IR) = RHO0
C
C------ default CG location is the input reference location
        PARVAL(IPXCG,IR) = XYZREF0(1)
        PARVAL(IPYCG,IR) = XYZREF0(2)
        PARVAL(IPZCG,IR) = XYZREF0(3)
C
        PARVAL(IPMASS,IR) = RMASS0
        PARVAL(IPIXX,IR) = RINER0(1,1)
        PARVAL(IPIYY,IR) = RINER0(2,2)
        PARVAL(IPIZZ,IR) = RINER0(3,3)
        PARVAL(IPIXY,IR) = RINER0(1,2)
        PARVAL(IPIYZ,IR) = RINER0(2,3)
        PARVAL(IPIZX,IR) = RINER0(3,1)
C
        PARVAL(IPCD0,IR) = CDREF0
C
        PARVAL(IPCLA,IR) = DCL_A0
        PARVAL(IPCLU,IR) = DCL_U0
        PARVAL(IPCMA,IR) = DCM_A0
        PARVAL(IPCMU,IR) = DCM_U0
C
        ITRIM(IR) = 0
        NEIGEN(IR) = 0
      ENDDO
C
C---- add control variables, direct constraints
      DO N = 1, NDMAX
        IV = IVTOT + N
        IC = ICTOT + N
        DO IR = 1, NRMAX
          ICON(IV,IR) = IC
          CONVAL(IC,IR) = 0.
        ENDDO
      ENDDO
C
C---- default number of run cases
      IRUN = 1
      NRUN = 1
C
C---- all run cases are targets for eigenmode calculation
      IRUNE = 0
C
C---- first run case is default for time march initial state
      IRUNT = 1
C
      RETURN
      END ! RUNINI



      SUBROUTINE RUNGET(LU,FNAME,ERROR)
C-------------------------------------------------
C     Reads run case file into run case arrays
C-------------------------------------------------
      INCLUDE 'AVL.INC'
      CHARACTER*(*) FNAME
      LOGICAL ERROR
C
      CHARACTER*80 LINE, REST
      CHARACTER*12 VARN, CONN
      CHARACTER*8  PARN
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=90)
      ILINE = 0
C
      IR = 0
C
C==============================================================
C---- start line-reading loop
 10   CONTINUE
C
      READ(LU,1000,END=50) LINE
 1000 FORMAT(A)
      ILINE = ILINE + 1
C
      KCOL = INDEX(LINE,':' )
      KARR = INDEX(LINE,'->')
      KEQU = INDEX(LINE,'=' )
      IF(KCOL.NE.0) THEN
C----- start of new run case
       READ(LINE(KCOL-3:KCOL-1),*,ERR=80) IR
C
       IF(IR.LT.1 .OR. IR.GT.NRMAX) THEN
        WRITE(*,*) 'RUNGET:  Run case array limit NRMAX exceeded:', IR
        IR = 0
        GO TO 10
       ENDIF
C
       NRUN = MAX(NRUN,IR)
C
       RTITLE(IR) = LINE(KCOL+1:80)
       CALL STRIP(RTITLE(IR),NRT)
C
      ELSEIF(IR.EQ.0) THEN
C----- keep ignoring lines if valid run case index is not set
       GO TO 10
C
      ELSEIF(KARR.NE.0 .AND. KEQU.NE.0) THEN
C----- variable/constraint declaration line
       VARN = LINE(1:KARR-1)
       CONN = LINE(KARR+2:KEQU-1)
       CALL STRIP(VARN,NVARN)
       CALL STRIP(CONN,NCONN)
C
       DO IV = 1, NVTOT
         IF(INDEX(VARNAM(IV),VARN(1:NVARN)).NE.0) GO TO 20
       ENDDO
       WRITE(*,*) 'Ignoring unrecognized variable: ', VARN(1:NVARN)
       GO TO 10
C
 20    CONTINUE
       DO IC = 1, NCTOT
         IF(INDEX(CONNAM(IC),CONN(1:NCONN)).NE.0) GO TO 25
       ENDDO
       WRITE(*,*) 'Ignoring unrecognized constraint: ', CONN(1:NCONN)
       GO TO 10
C
 25    CONTINUE
       READ(LINE(KEQU+1:80),*,ERR=80) CONV
C
       ICON(IV,IR) = IC
       CONVAL(IC,IR) = CONV
C
      ELSEIF(KARR.EQ.0 .AND. KEQU.NE.0) THEN
C----- run case parameter data line
       PARN = LINE(1:KEQU-1)
       CALL STRIP(PARN,NPARN)
       DO IP = 1, NPTOT
         IF(INDEX(PARNAM(IP),PARN(1:NPARN)).NE.0) GO TO 30
       ENDDO
       WRITE(*,*) 'Ignoring unrecognized parameter: ', PARN(1:NPARN)
       GO TO 10
C
 30    CONTINUE
       REST = LINE(KEQU+1:80)
       READ(REST,*,ERR=80) PARV
       PARVAL(IP,IR) = PARV


       if(.false.) then
       CALL STRIP(REST,NREST)
       KBLK = INDEX(REST,' ')
       IF(KBLK .NE. 0) THEN
        REST = REST(KBLK+1:80)
        CALL STRIP(REST,NREST)
        IF(NREST.GT.0) THEN
         PARUNCH(IP) = REST
        ENDIF
       ENDIF
       endif

      ENDIF
C
C---- keep reading lines
      GO TO 10
C
C==============================================================
C
 50   CONTINUE
      CLOSE(LU)
      ERROR = .FALSE.
      RETURN
C
 80   CONTINUE
      CALL STRIP(FNAME,NFN)
      CALL STRIP(LINE ,NLI)
      WRITE(*,8000) FNAME(1:NFN), ILINE, LINE(1:NLI)
 8000 FORMAT(/' Run case file  ',A,'  read error on line', I4,':',A)
      CLOSE(LU)
      ERROR = .TRUE.
      NRUN = 0
      RETURN
C
 90   CONTINUE
      CALL STRIP(FNAME,NFN)
      WRITE(*,9000) FNAME(1:NFN)
 9000 FORMAT(/' Run case file  ',A,'  open error')
      ERROR = .TRUE.
      RETURN
      END ! RUNGET



      SUBROUTINE RUNSAV(LU)
      INCLUDE 'AVL.INC'
C
      DO IR = 1, NRUN
        WRITE(LU,1010) IR, RTITLE(IR)
        DO IV = 1, NVTOT
          IC = ICON(IV,IR)
          WRITE(LU,1050) VARNAM(IV), CONNAM(IC), CONVAL(IC,IR)
        ENDDO
C
        WRITE(LU,*)
C
        DO IP = 1, NPTOT
          WRITE(LU,1080) PARNAM(IP), PARVAL(IP,IR), PARUNCH(IP)
        ENDDO
      ENDDO
C
 1010 FORMAT(/' ---------------------------------------------'
     &       /' Run case', I3,':  ', A /)
 1050 FORMAT(1X,A,' ->  ', A, '=', G14.6, 1X, A)
 1080 FORMAT(1X,A,'=', G14.6, 1X, A)
C
      RETURN
      END ! RUNSAV



      LOGICAL FUNCTION LOWRIT(FNAME)
      CHARACTER*(*) FNAME
C
      CHARACTER*1 ANS
 1000 FORMAT(A)
C
      K = INDEX(FNAME,' ')
C
      WRITE(*,*) 'File  ', FNAME(1:K), ' exists.  Overwrite?  Y'
      READ (*,1000) ANS
      LOWRIT = INDEX('Nn',ANS) .EQ. 0
C
      RETURN
      END


      SUBROUTINE AOCFIL(FNAME,IFILE)
      CHARACTER*(*) FNAME
C
      CHARACTER*1 ANS
 1000 FORMAT(A)
C
      K = INDEX(FNAME,' ')
C
      WRITE(*,*) 'File  ', FNAME(1:K), 
     &     ' exists.  Append, Overwrite, or Cancel?  A'
      READ (*,1000) ANS
      IFILE = INDEX('AOC',ANS) + INDEX('aoc',ANS)
C
      IF(IFILE.EQ.0) IFILE = 1
C
      RETURN
      END

