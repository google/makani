C***********************************************************************
C    Module:  aplotvl.f
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

      SUBROUTINE PLOTVL(AZIM, ELEV, TILT, RINV)
C
C----Plotting module for AVL vortex lattice program
C
C    Plots geometry and loading results for vortex lattice
C     Geometry plots:
C       Surfaces     (straight tapered panels of chordwise strips of vortices)
C       Strips       (chordwise strips of vortices)
C       Vortex legs  (bound vortex legs)
C       Control pts  (vortex element control points)
C       Camber slope (camber of each strip - used for establishing BC's)
C       Hinge lines  (surface deflection axis and deflected surface outline)
C       Strip loading (chordwise plot of vortex loading on each strip)
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      LOGICAL LKEYS, LCOLHC, ERROR
      LOGICAL LINITVIEW, LDEBUG
      SAVE    LINITVIEW, LDEBUG
      CHARACTER*80 LINE
      CHARACTER*4 OPT
      CHARACTER*1 CKEY
C
      REAL RINP(10), ARG(5)
C
C---- viewpoint changes (deg), zoom/unzoom, perspective scale factors
      DATA DAZIM, DELEV, ZUFAC, DPAN , PRAT 
     &     / 5.0 , 5.0 , 1.25 , 0.075, 1.1 /
C
    1 FORMAT(A)
C
C***************************************************
C---- Initialization for plot program variables
      IF(LPLTNEW .OR. (.NOT.LPLOT)) THEN 
        LPLTNEW = .FALSE.
        LINITVIEW = .FALSE.
        LDEBUG = .FALSE.
      ENDIF
C
      LKEYS = .FALSE.
C
C---- find geometry and axis limits
      CALL GLIMS(GMIN,GMAX,.FALSE.)
      CALL AXLIMS
C
C***************************************************
C---- Setup view transformation 
 4    CONTINUE
      CALL VIEWINIT(AZIM, ELEV, TILT, RINV)
C
C---- scale and project x,y,z orientation arrows UAXARW(....)
C-    also project x,y,z unit vectors UAXDIR(..)
      ARWLEN = 0.2*BREF
C
      RHEADP = RHEAD*ARWLEN
      DO IXYZ = 1, 3
        DO K = 1, 3
          DO IL = 1, NLINAX
            UAXARWP(K,1,IL,IXYZ) = UAXARW(K,1,IL,IXYZ)*ARWLEN
            UAXARWP(K,2,IL,IXYZ) = UAXARW(K,2,IL,IXYZ)*ARWLEN
          ENDDO
        ENDDO
      ENDDO
      NLIN = 3 * 2*NLINAX
      NAXD = 3   
      CALL VIEWPROJ(UAXARWP,NLIN,UAXARWP)
      CALL VIEWPROJ(UAXDIR ,NAXD,UAXDIRP)
C
      DO IAXD = 1, NAXD
        DIRMAG = SQRT( UAXDIRP(1,IAXD)**2
     &               + UAXDIRP(2,IAXD)**2
     &               + UAXDIRP(3,IAXD)**2 )
        IF(DIRMAG .GT. 0.0) THEN
         UAXDIRP(1,IAXD) = UAXDIRP(1,IAXD)/DIRMAG
         UAXDIRP(2,IAXD) = UAXDIRP(2,IAXD)/DIRMAG
         UAXDIRP(3,IAXD) = UAXDIRP(3,IAXD)/DIRMAG
        ENDIF
      ENDDO
C
C---- Setup hidden line data
      CALL HIDINIT(.TRUE.)
C
C---- find plot limits
      CALL GLIMS(GMINP,GMAXP,.TRUE.)
      XMIN = GMINP(1)
      YMIN = GMINP(2)
      XMAX = GMAXP(1)
      YMAX = GMAXP(2)
C
C---- set plot offsets and scale
      CALL OFFINI
C
      LINITVIEW = .FALSE.
C
C
C***************************************************
C---- start geometry plot
 6    CONTINUE
      CALL PVLINI(TITLE,AZIM,ELEV,TILT,VERSION,.FALSE.)

C---- DEBUG: Check the triangle array
      IF(LDEBUG) THEN
       CALL GETCOLOR(ICOL0)
       CALL NEWCOLORNAME('RED')
       DO I = 1, NTRI
        CALL PLTPOLY(TRI(1,I),3)
       END DO
       CALL NEWCOLOR(ICOL0)
       CALL PLFLUSH
cc      PAUSE
      ENDIF
C
C---- plot the selected geometry items
      CALL PLOTGEOM


      CALL PLFLUSH
C
 7    CONTINUE
C
      IF(LKEYS) THEN
C------ process keystroke-mode commands, skipping normal menu loop farther below
        CKEY = ' '
        CALL GETCURSORXY(XX,YY,CKEY)
C
        IF(CKEY.EQ.' ') THEN
C------- exit to normal menu loop
         LKEYS = .FALSE.
C
        ELSEIF(INDEX('Ll',CKEY).NE.0) THEN
         AZIM = AZIM + DAZIM
         IF(AZIM .GT.  180.01) AZIM = AZIM - 360.0
         GO TO 4
C
        ELSEIF(INDEX('Rr',CKEY).NE.0) THEN
         AZIM = AZIM - DAZIM
         IF(AZIM .LT. -180.01) AZIM = AZIM + 360.0
         GO TO 4
C
        ELSEIF(INDEX('Uu',CKEY).NE.0) THEN
         ELEV = ELEV + DELEV
         IF(ELEV .GT.  90.01) THEN
          ELEV = 90.0
          WRITE(*,*) 'Elevation angle is limited to +/- 90 deg'
         ENDIF
         GO TO 4
C
        ELSEIF(INDEX('Dd',CKEY).NE.0) THEN
         ELEV = ELEV - DELEV
         IF(ELEV .LT. -90.01) THEN
          ELEV = -90.0
          WRITE(*,*) 'Elevation angle is limited to +/- 90 deg'
         ENDIF
         GO TO 4
C
c        ELSEIF(INDEX('Tt',CKEY).NE.0) THEN
c         TILT = TILT + 5.0
c         IF(TILT .GT.  180.01) TILT = TILT - 360.0
c         GO TO 4
cC
c        ELSEIF(INDEX('Ss',CKEY).NE.0) THEN
c         TILT = TILT - 5.0
c         IF(TILT .LT. -180.01) TILT = TILT + 360.0
c         GO TO 4
C
        ELSEIF(INDEX('Cc',CKEY).NE.0) THEN
         AZIM = 0.
         ELEV = 0.
         TILT = 0.
         GO TO 4
C
        ELSEIF(INDEX('Zz',CKEY).NE.0) THEN
         XOFF = XX/SF + XOFF
         YOFF = YY/SF + YOFF
         SF = SF*ZUFAC
         XDIF =    1.0/SF
         YDIF = PLOTAR/SF
         XOFF = XOFF - 0.5*XDIF
         YOFF = YOFF - 0.5*YDIF
         GO TO 6
C
        ELSEIF(INDEX('Ee',CKEY).NE.0) THEN
         XOFF = XX/SF + XOFF
         YOFF = YY/SF + YOFF
         SF = SF/ZUFAC
         XDIF =    1.0/SF
         YDIF = PLOTAR/SF
         XOFF = XOFF - 0.5*XDIF
         YOFF = YOFF - 0.5*YDIF
         GO TO 6
C
        ELSEIF(INDEX('Pp',CKEY).NE.0) THEN
         DX = XX - 0.5
         DY = YY - 0.5*PLOTAR
C
         XOFF = XOFF + DPAN*DX/SF
         YOFF = YOFF + DPAN*DY/SF
         GO TO 6
C
        ELSEIF(INDEX('Nn',CKEY).NE.0) THEN
         GO TO 4
C
        ELSEIF(INDEX('Ii',CKEY).NE.0) THEN
         IF(RINV.EQ.0.0) THEN
          RINV = 0.02/BREF
         ELSE
          RINV = RINV*PRAT
         ENDIF
         WRITE(*,8010) UNITL/RINV, UNCHL(1:NUL)
         GO TO 4
C
        ELSEIF(INDEX('Oo',CKEY).NE.0) THEN
         IF(RINV .LT. 0.02/BREF) THEN
          RINV = 0.0
          WRITE(*,*) '  Observer distance = infinity  (no perspective)'
         ELSE
          RINV = RINV/PRAT
          WRITE(*,8010) UNITL/RINV, UNCHL(1:NUL)
         ENDIF
         GO TO 4
C
        ELSEIF(INDEX('Aa',CKEY).NE.0) THEN
C------- Annotate
         CALL ANNOT(CH)
         WRITE(*,2040)
         GO TO 7
C
        ELSEIF(INDEX('Hh',CKEY).NE.0) THEN
C------- Make hardcopy
         IF(LPLOT) CALL PLEND
         LPLOT = .FALSE.
         CALL REPLOT(IDEVH)
         GO TO 6
C
        ELSE
         WRITE(*,*)
         WRITE(*,*) '*** "', CKEY,'"  key not recognized'
         WRITE(*,2040)
C
        ENDIF
C
        GO TO 4
      ENDIF
C
C
C***************************************************
C---- top of normal user interaction loop
C
 8    CONTINUE
      WRITE(*,2010)
      WRITE(*,2020) LCHORDLINE,LCAMBER,LCNTLPTS,LWAKEPLT,
     &              LBOUNDLEG,LNRMLPLT,LLOADPLT,LAXESPLT
      WRITE(*,2030) 
      READ(*,1) OPT 
C
 2010 FORMAT(
     &  /' ========================================='
     &  /'  K eystroke mode       V iewpoint        '
     &  /'  A nnotate plot        O ptions          '
     &  /'  H ardcopy plot        S elect surfaces  '
     &  /'  Z oom                 U nzoom           ')
 2020 FORMAT(
     &  /'  CH ordline   ',L2,7X,'CA amber       ',L2,
     &  /'  CN tlpoint   ',L2,7X,'TR ailing legs ',L2,
     &  /'  BO ound leg  ',L2,7X,'NO rmal vector ',L2,
     &  /'  LO ading     ',L2,7X,'AX es, xyz ref.',L2 )
 2030 FORMAT(/' Geometry plot command: ', $)
C
      CALL TOUPER(OPT) 
      IF(OPT.EQ.'  ' .OR. OPT.EQ.'Q ') THEN
       CALL PLEND
       RETURN
      ENDIF
C
      IF(OPT.EQ.'K ') THEN
C----- enter keystroke mode
       LKEYS = .TRUE.
       WRITE(*,2040)
 2040  FORMAT(
     &   /'  ------------------------------------------------'
     &   /'  Type keys in graphics window...'
     &  //'    L eft               R ight        (Azimuth  ) '
     &   /'    U p                 D own         (Elevation) '
     &   /'    C lear'
     &  //'    P an from cursor    Z oom on cursor           '
     &   /'    I ngress            E xpand on cursor         '
     &   /'    O utgress           N ormal size              '
     &  //'    H ardcopy           A nnotate plot            '
     &  //'  ...<space> to exit  '
     &   /'  ------------------------------------------------')
       GO TO 6
      ENDIF
C
C===================================================
      IF(OPT.EQ.'V ') THEN
 20     WRITE(*,2050) AZIM, ELEV
 2050   FORMAT(
     &   /'  Enter viewpoint azimuth,elevation angles:',2F7.0)
        RINP(1) = AZIM
        RINP(2) = ELEV
        NINP = 2
        CALL READR(NINP,RINP,ERROR)
        IF(ERROR) GO TO 20
C
        AZIM = RINP(1)
        ELEV = RINP(2)
        IF(AZIM .GT.  180.01) AZIM = AZIM - 360.0
        IF(AZIM .LT. -180.01) AZIM = AZIM + 360.0
        IF(ELEV .GT.   90.01) ELEV =  90.0
        IF(ELEV .LT.  -90.01) ELEV = -90.0
        GO TO 4
C===================================================
      ELSE IF(OPT.EQ.'CH') THEN
C------ Toggle chordline plotting
        LCHORDLINE = .NOT.LCHORDLINE
C
C===================================================
      ELSE IF(OPT.EQ.'CA') THEN
C------ Toggle camber
        LCAMBER = .NOT.LCAMBER
C
C===================================================
      ELSE IF(OPT.EQ.'BO') THEN
C------ Toggle plotting of bound legs
        LBOUNDLEG = .NOT.LBOUNDLEG
C
C===================================================
      ELSE IF(OPT.EQ.'CN') THEN
C------ Toggle control point plotting
        LCNTLPTS = .NOT.LCNTLPTS
C
C===================================================
      ELSE IF(OPT.EQ.'TR') THEN
C------ Toggle wake plotting
        LWAKEPLT = .NOT.LWAKEPLT
C
C===================================================
      ELSE IF(OPT.EQ.'LO') THEN
C------ Toggle plotting of surface loading
        LLOADPLT = .NOT.LLOADPLT
C
C===================================================
      ELSE IF(OPT.EQ.'NO') THEN
C------ Toggle plotting of surface normals
        LNRMLPLT = .NOT.LNRMLPLT
C
C===================================================
      ELSE IF(OPT.EQ.'AX') THEN
C------ Toggle plotting of x,y,z axes
        LAXESPLT = .NOT.LAXESPLT
C
C===================================================
      ELSE IF(OPT.EQ.'RE') THEN
C------ Toggle plotting of x,y,z reference location
        LRREFPLT = .NOT.LRREFPLT
C
C===================================================
cc    ELSE IF(OPT.EQ.'HI') THEN
ccC---- Toggle plotting of hinge axes 
cc      LHINGEPLT = .NOT.LHINGEPLT
C
C===================================================
      ELSE IF(OPT.EQ.'DE') THEN
C------ Toggle DEBUG
        LDEBUG = .NOT.LDEBUG
C
C===================================================
      ELSEIF(OPT.EQ.'A ') THEN
C----- Annotate
       IF(LPLOT) THEN
        CALL ANNOT(CH)
       ELSE
        WRITE(*,*) 'No active plot'
       ENDIF
C
C===================================================
      ELSEIF(OPT.EQ.'H ') THEN
C----- Make hardcopy
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVH)
       GO TO 8
C
C===================================================
      ELSEIF(OPT.EQ.'Z ') THEN
C----- Zoom in on the plot
       IF (LPLOT) THEN
        CALL OFFGET   
       ELSE
        WRITE(*,*) 'No active plot'
       ENDIF
C
C===================================================
      ELSEIF(OPT.EQ.'U ') THEN
C----- Reset to default offset and scaling factors
       CALL OFFINI   
C
C===================================================
      ELSEIF(OPT.EQ.'S ') THEN
C----- Set surfaces to plot
       CALL SELSURF(AZIM, ELEV, TILT, RINV)
       GO TO 4
C
C===================================================
      ELSEIF(OPT.EQ.'O ') THEN
C----- Display options 
 50    CONTINUE
       WRITE(*,*) ' '
       ROB = 999999.
       IF(RINV.NE.0.) ROB = 1.0/RINV
       LCOLHC = IDEVH.EQ.4
       WRITE(*,5500) LHID, ROB, SIZE, CPFAC, ENFAC, LCOLHC,
     &              LABEL_SURF, LABEL_STRP, LABEL_VRTX, LABEL_BODY
 5500  FORMAT(/' ================================================'
     &        /'   H iddenLine            currently ',L2
     &        /'   D ist. to observer     currently ',F12.2
     &        /'   S ize of plot          currently ',F10.2
     &        /'   L oading scale factor  currently ',F10.4
     &        /'   N ormal  scale factor  currently ',F10.4
     &        /'   C olor hardcopy        currently ',L2
     &        /'   LN label surfaces      currently ',L2
     &        /'   LS label strips        currently ',L2
     &        /'   LV label vortices      currently ',L2
     &        /'   LB label bodies        currently ',L2
     &       //' Select item to change (or <return>):  ', $)
       READ(*,1,ERR=50) LINE
       CALL STRIP(LINE,NLIN)
       OPT = LINE(1:2)
       CALL TOUPER(OPT)
C
       LINE(1:2) = '  '
       CALL STRIP(LINE,NLIN)
       NARG = 1
       CALL GETFLT(LINE,ARG,NARG,ERROR)
C
C----------------------------------------------------
       IF(OPT.EQ.'  ') THEN
         IF(LINITVIEW) THEN
           GO TO 4
          ELSE
           GO TO 6
         ENDIF
C
C----------------------------------------------------
       ELSE IF (OPT.EQ.'S ') THEN 
C------ Change size
        IF(NARG.GT.0) THEN
         SIZE = MAX( ARG(1) , 0.0 )
        ELSE
   52    WRITE(*,1052)
 1052    FORMAT(' Enter new plot size:  ',$)
         READ(*,*,ERR=52,END=50) SIZE
        ENDIF
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'D ') THEN
C------ Set distance to observer (controls perspective)
        IF(NARG.GT.0) THEN
         ROB = MAX( ARG(1) , 0.0 )
        ELSE
   53    WRITE(*,1053)
 1053    FORMAT(' Enter new distance to observer (0 for infinity):  ',$)
         READ (*,*,ERR=53,END=50) ROB
        ENDIF
C
        IF(ROB.EQ.0.0) THEN
         RINV = 0.
        ELSE
         RINV = 1.0/ROB
        ENDIF
        LINITVIEW = .TRUE.
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'L ') THEN
C------ Loading scale factor for DCP values (as fraction of CREF)
        IF(NARG.GT.0) THEN
         CPFAC = ARG(1)
        ELSE
   54    WRITE(*,1054)
 1054    FORMAT(' Enter new loading scale factor:  ',$)
         READ (*,*,ERR=54,END=50) CPFAC
        ENDIF
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'N ') THEN
C------ Normal vector scale factor (as fraction of CREF)
        IF(NARG.GT.0) THEN
         ENFAC = ARG(1)
        ELSE
 55      WRITE(*,1055)
 1055    FORMAT(' Enter new normal scale factor:  ',$)
         READ (*,*,ERR=55,END=50) ENFAC
        ENDIF
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'H ') THEN
C------ Toggle hidden line
        LHID = .NOT.LHID
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'C ') THEN
C------ Toggle Color hardcopy
        LCOLHC = .NOT.LCOLHC
        IF(LCOLHC) THEN
         IDEVH = 4
        ELSE
         IDEVH = 2
        ENDIF
C
C----------------------------------------------------
       ELSE IF(OPT.EQ.'LN') THEN
C------ Labeling for surfaces
        LABEL_SURF = .NOT.LABEL_SURF
C----------------------------------------------------
       ELSE IF(OPT.EQ.'LS') THEN
C------ Labeling for strips
        LABEL_STRP = .NOT.LABEL_STRP
C----------------------------------------------------
       ELSE IF(OPT.EQ.'LV') THEN
C------ Labeling for vortex elements
        LABEL_VRTX = .NOT.LABEL_VRTX
C----------------------------------------------------
       ELSE IF(OPT.EQ.'LB') THEN
C------ Labeling for bodies
        LABEL_BODY = .NOT.LABEL_BODY
       ENDIF
       GO TO 50
C
      ELSE
       WRITE(*,*) ' * Unrecognized command'
C
      ENDIF
C
      GO TO 6
C
 8010 FORMAT('   Observer distance =', G11.3,1X,A)
C
      END ! PLOTVL


      SUBROUTINE SELSURF(AZIM, ELEV, TILT, RINV)
C--- Selects surfaces for plotting
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      CHARACTER*40 OPT, OPT2
      CHARACTER*1 CKEY
      LOGICAL ERROR
C
    1 FORMAT(A)
C
C---- Select surfaces to plot
   60 CONTINUE
      CALL VIEWINIT(AZIM, ELEV, TILT, RINV)
      CALL HIDINIT(.TRUE.)
      CALL GLIMS(GMIN,GMAX,.FALSE.)
      CALL AXLIMS
cc    CALL OFFINI
      CALL PVLINI(TITLE,AZIM,ELEV,TILT,VERSION,.FALSE.)
      CALL PLOTGEOM
      CALL PLFLUSH
C
      WRITE(*,*) ' '
      WRITE(*,*) '==================================================='
      WRITE(*,*) '#surf component Nchord Nspan   plot?   Surface name'
      DO N = 1, NSURF
        WRITE(*,61) N, LSCOMP(N), NK(N), NJ(N), LPLTSURF(N), STITLE(N)
      END DO
   61 FORMAT(1X,I3,4X,I3, 4X,I3,4X,I3,4X,L3,6X,A)
C

 62   WRITE(*,*) '---------------------------------------------------'
      WRITE(*,1062) IMARKSURF
 1062 FORMAT(/'   #    Select surface #  or  #:#   selects range',
     &       /'  -#  DeSelect surface #  or -#:# deselects range,'
     &       /'   A    Select all surfaces',
     &       /'   N  DeSelect all surfaces',
     &       /'   M    Mark surface, currently', I3 )
C
 65   WRITE(*,1065)
 1065 FORMAT(/' Enter selection: (P prints surface list):  ',$)
C
      READ(*,1,ERR=65) OPT
cc      CALL LC2UC(OPT)
      KC = INDEX(OPT,':')
      KM = INDEX(OPT,',')
C
      CKEY = OPT(1:1)
C
      IF(CKEY.EQ.' ') THEN
        GO TO 100
C
C--- Read colon-separated number specs - i.e. 1:3 or -5:8
       ELSEIF(KC.GT.1) THEN
        READ(OPT(1:KC-1),*,ERR=62) N1
        READ(OPT(KC+1:LEN(OPT)),*,ERR=62) N2
        L1 = ABS(N1)
        L2 = ABS(N2)
        L1 = MIN(L1,NSURF)
        L2 = MIN(L2,NSURF)
        IF(L1.GT.0 .AND. L2.GE.L1) THEN
          DO N = L1, L2
            LPLTSURF(N) = (N1.GT.0)
          END DO
        ENDIF
C
C--- Read comma-separated number string - i.e. 1,2,3,-5,-8
       ELSEIF(KM.GT.1) THEN
        OPT2 = OPT
 67     READ(OPT2(1:KM-1),*,ERR=62) N
        NABS = ABS(N)
        IF(NABS.LE.0 .OR. NABS.GT.NSURF) GO TO 65
        LPLTSURF(NABS) = (N.GT.0) 
        OPT2 = OPT2(KM+1:LEN(OPT2))
        KM = INDEX(OPT2,',')
        IF(KM.GT.1) GO TO 67
C
        READ(OPT2,*,ERR=62) N
        NABS = ABS(N)
        IF(NABS.LE.0 .OR. NABS.GT.NSURF) GO TO 65
        LPLTSURF(NABS) = (N.GT.0) 
C
      ELSEIF(INDEX('Pp',CKEY).NE.0) THEN
        GO TO 60

      ELSEIF(INDEX('Aa',CKEY).NE.0) THEN
        DO N=1,NSURF
          LPLTSURF(N) = .TRUE.
        END DO
C
      ELSEIF(INDEX('Nn',CKEY).NE.0) THEN
        DO N=1,NSURF
          LPLTSURF(N) = .FALSE.
        END DO
C
      ELSEIF(INDEX('Mm',CKEY).NE.0) THEN
        NINP = 1
        CALL GETINT(OPT(2:40),IMARKSURF,NINP,ERROR)
C
        IF(NINP.EQ.0 .OR. ERROR) THEN
         WRITE(*,1068)
 1068    FORMAT(/' Enter surface # to mark:  ', $)
         READ (*,*,ERR=65,END=65) IMARKSURF
        ENDIF
C
        IF(IMARKSURF.LT.0)     IMARKSURF = 0
        IF(IMARKSURF.GT.NSURF) IMARKSURF = 0
C
      ELSE
        READ(OPT,*,ERR=65) N
        NABS = ABS(N)
        IF(NABS.LE.0 .OR. NABS.GT.NSURF) GO TO 65
        LPLTSURF(NABS) = (N.GT.0) 
      ENDIF
      GO TO 60
CC
 100  RETURN
      END ! SELSURF




      SUBROUTINE PVLINI(TITLE,AZIM,ELEV,TILT,VERSION,LDEVM)
C...PURPOSE:    Initializes plot frame for plotting and 
C               puts up axis orientation and labels
C
      INCLUDE 'AVLPLT.INC'
C
      CHARACTER*80 TITLE
      LOGICAL LDEVM

      CHARACTER*1 CHXYZ(3)
      DATA CHXYZ / 'x', 'y', 'z' /
C
      IF (LPLOT) CALL PLEND
C
      SCH = 0.7*SIZE*CH
      DLEN = 0.045*SIZE
C
      IF(LDEVM) THEN
       CALL PLTINI(IDEVM)
      ELSE
       CALL PLTINI(IDEV)
      ENDIF

C
C      IF(.NOT.LPLOT) CALL COLORMAP(NCOLORS)
      LPLOT = .TRUE. 
      XPLT = PMARG
      YPLT = PMARG
      CALL PLCHAR(XPLT,YPLT,0.6*SCH,'AVL ',0.0,4)
      CALL PLNUMB(999.,999.,0.6*SCH,VERSION,0.0,2)
C
      CALL GETCOLOR(ICOL0)
C
C---- small axis arrows
      CALL NEWPEN(2)
      CALL NEWCOLORNAME('GREEN')
      XO = PMARG + DLEN + 1.5*SCH
      YO = PMARG + DLEN + 8.5*SCH
C
      AFAC = DLEN/ARWLEN
      DO IXYZ = 1, 3
        CALL ARWPLT(XO,YO,AFAC,
     &              UAXARWP(1,1,1,IXYZ),
     &              UAXDIRP(1,IXYZ),RHEADP,NHEAD)
      ENDDO
C
C---- xyz labels
      LTIP = NHEAD+1
      DO IXYZ = 1, 3
        XPLT = XO + UAXARWP(1,2,LTIP,IXYZ)*AFAC
     &            + UAXDIRP(1       ,IXYZ)*1.3*SCH - 0.5*SCH
        YPLT = YO + UAXARWP(2,2,LTIP,IXYZ)*AFAC
     &            + UAXDIRP(2       ,IXYZ)*1.3*SCH - 0.5*SCH
        CALL PLCHAR(XPLT,YPLT,0.9*SCH,CHXYZ(IXYZ),0.0,1)
      ENDDO
      CALL NEWCOLOR(ICOL0)
C
C---- azimuth and elevation angles
      CALL NEWPEN(2)
      XPLT = PMARG
      YPLT = PMARG + 3.0*SCH
C
      CALL PLCHAR(XPLT             ,YPLT,0.75*SCH,'Elev',0.0, 4)
      CALL PLCHAR(XPLT+4.5*0.75*SCH,YPLT,0.75*SCH,'='   ,0.0, 1)
      CALL PLNUMB(XPLT+6.0*0.75*SCH,YPLT,0.75*SCH,ELEV  ,0.0,-1)
      CALL PLMATH(999.             ,999.,0.75*SCH,'"'   ,0.0, 1)
C
      YPLT = YPLT + 1.8*SCH
      CALL PLCHAR(XPLT             ,YPLT,0.75*SCH,'Azim',0.0, 4)
      CALL PLCHAR(XPLT+4.5*0.75*SCH,YPLT,0.75*SCH,'='   ,0.0, 1)
      CALL PLNUMB(XPLT+6.0*0.75*SCH,YPLT,0.75*SCH,AZIM  ,0.0,-1)
      CALL PLMATH(999.             ,999.,0.75*SCH,'"'   ,0.0, 1)
C
C---- Configuration title
      CALL NEWPEN(3)
      XPLT = PMARG + 10.0*SCH
      YPLT = PMARG
      CALL PLCHAR(XPLT,YPLT,1.1*SCH,TITLE,0.0,80)
C
C---- Move plot origin to miss axis block
      CALL PLOT(PMARG+2.0*DLEN+2.0*SCH,PMARG+5.0*SCH,-3)   
      CALL NEWFACTOR(SIZE)
      CALL NEWPEN(1)
C
C      CALL PLOT(0.,0.,3)   
C      CALL PLOT(1.,0.,2)   
C      CALL PLOT(1.,AR,2)   
C      CALL PLOT(0.,AR,2)   
C      CALL PLOT(0.,0.,2)   
C
      RETURN        
      END ! PVLINI  



      SUBROUTINE PLOTGEOM
C...PURPOSE:    Plotting routine to plot selected geometry data
C
C     Plots:
C       Surfaces     (straight tapered panels of chordwise strips of vortices)
C       Strips       (chordwise strips of vortices)
C       Vortex legs  (bound vortex legs)
C       Control pts  (vortex element control points)
C       Normal vector (normal vector at control points)
C       Camber slope (camber of each strip - used for establishing BC's)
C       Hinge lines  (surface deflection axis and deflected surface outline)
C       Strip loading (chordwise plot of vortex loading on each strip)
C
C     User can optionally display an identifying index for each item 
C      (surface, strip, element)
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      LOGICAL LVIS
C
      REAL ENTOT(3)
      REAL PLAB(3), PLAB2(3), SVEC1(3), SVEC2(3)
      REAL PTSORG(3)
      REAL PTS_LINES(3,2,NVMAX), ID_LINES(NVMAX),
     &     PTS_LPROJ(3,2,NVMAX)
      REAL PTS_POINT(3,NLMAX),
     &     PTS_PPROJ(3,NLMAX),
     &     RNUM_POINT(NLMAX)
      LOGICAL LOFF(NLMAX)
      INTEGER IDUM(1)
      REAL DUM(1,1)

      DATA ID_LINES / NVMAX * 0.0 /
C
      INCLUDE 'MASKS.INC'
C
      EPS    = 1.0E-5
      CHSIZE = 0.01
C
C---- number of line segments for drawing a body section "hoop"
      KK = 18
C
      IF(LAXESPLT) THEN
C----- plot x,y,z axes
       DTICK = 0.08 * MIN( AXDEL(1), AXDEL(2), AXDEL(3) )
       CHN = 1.0*DTICK*SF
C
       IP = 0
       IN = 0
C
C----- go over x,y,z axes
       DO 50 IAX = 1, 3
C------- go over axis tick marks
         DO IANN = 1, NAXANN(IAX)
C--------- previous and current tick mark location (x, y, or z value)
           XYZM = AXMIN(IAX) + AXDEL(IAX)*FLOAT(IANN-2)
           XYZO = AXMIN(IAX) + AXDEL(IAX)*FLOAT(IANN-1)
C
C--------- go over x,y,z directions
           DO 505 IDIR = 1, 3
             IF(IANN.EQ.1 .AND. IDIR.EQ.IAX) GO TO 505
C
             IF(IDIR.EQ.IAX) THEN
C------------ axis segment, along IAX direction
              IP = IP+1
              DO K = 1, 3
                PTS_LINES(K,1,IP) = 0.
                PTS_LINES(K,2,IP) = 0.
              ENDDO
              PTS_LINES(IAX,1,IP) = XYZM
              PTS_LINES(IAX,2,IP) = XYZO
C
             ELSE
C------------ tick marks, along the other two directions
              IP = IP+1
              DO K = 1, 3
                PTS_LINES(K,1,IP) = 0.
                PTS_LINES(K,2,IP) = 0.
              ENDDO
              PTS_LINES(IAX ,1,IP) = XYZO
              PTS_LINES(IAX ,2,IP) = XYZO
              PTS_LINES(IDIR,1,IP) = -0.5*DTICK
              PTS_LINES(IDIR,2,IP) =  0.5*DTICK
             ENDIF
 505       CONTINUE
C
C--------- save point on axis for annotation number placement
           IF(XYZO .NE. 0.0) THEN
            IN = IN+1
            RNUM_POINT(IN) = XYZO
            IF    (IAX.EQ.1) THEN
             PTS_POINT(1,IN) = XYZO
             PTS_POINT(2,IN) = 0.
             PTS_POINT(3,IN) = -0.5*DTICK - 1.3*CHN/SF
             LOFF(IN) = .FALSE.
            ELSEIF(IAX.EQ.2) THEN
             PTS_POINT(1,IN) = 0.
             PTS_POINT(2,IN) = XYZO
             PTS_POINT(3,IN) = -0.5*DTICK - 1.3*CHN/SF
             LOFF(IN) = .FALSE.
            ELSE
             PTS_POINT(1,IN) = 0.
             PTS_POINT(2,IN) = 0.
             PTS_POINT(3,IN) = XYZO
             LOFF(IN) = .TRUE.
            ENDIF
ccc            ID_LINES(IN) = IAX
            ID_LINES(IN) = 0
           ENDIF
C
         ENDDO
 50    CONTINUE
C
C-----------------------------------
C----- plot axes
       CALL NEWPEN(1)
       CALL GETCOLOR(ICOL0)
       CALL NEWCOLORRGB(0,128,0)
C
       NLINES = IP
       NPROJ = 2*NLINES
       CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
       CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C

       NPOINT = IN
       CALL VIEWPROJ(PTS_POINT,NPOINT,PTS_PPROJ)
C
       DO IN = 1, NPOINT
         LVIS = .TRUE.
         IF(LHID) THEN
          ID = 0
          NGRP = 0
          IDUM(1) = 0
          DUM(1,1) = 0.
          CALL HIDPNT(PTS_PPROJ(1,IN),ID,NGRP,IDUM,DUM,NTRI,TRI,LVIS)
         ENDIF
C
         RNUM = RNUM_POINT(IN)
         IF(LVIS) CALL PLTFLT(PTS_PPROJ(1,IN),RNUM,CHN,LOFF(IN),-2)
       ENDDO
C
C
C----- plot moment-reference point
       CALL NEWPEN(2)
       CALL NEWCOLORNAME('RED')
       IP = 0
       DO IDIR = 1, 3
         IP = IP+1
         PTS_LINES(1,1,IP) = XYZREF(1)
         PTS_LINES(2,1,IP) = XYZREF(2)
         PTS_LINES(3,1,IP) = XYZREF(3)
         PTS_LINES(IDIR,1,IP) = PTS_LINES(IDIR,1,IP) - 0.7*DTICK
         ID_LINES(IP) = 0
C
         PTS_LINES(1,2,IP) = XYZREF(1)
         PTS_LINES(2,2,IP) = XYZREF(2)
         PTS_LINES(3,2,IP) = XYZREF(3)
         PTS_LINES(IDIR,2,IP) = PTS_LINES(IDIR,2,IP) + 0.7*DTICK
         ID_LINES(IP) = 0
       ENDDO
C
       NLINES = IP
       NPROJ = 2*NLINES
       CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
       CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C
C
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
C
C---- go over surfaces
      DO 100 N = 1, NSURF
C
C------ do only surfaces which are to be plotted
        IF(.NOT.LPLTSURF(N)) GO TO 100
C
C------ Emphasize panel 'IMARKSURF' with color
        IF(IMARKSURF.EQ.N) THEN
          CALL NEWCOLORNAME('RED')
         ELSE
          CALL NEWCOLOR(ICOL0)
        ENDIF
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JINC = 1
        IF(IMAGS(N).LT.0) THEN
         J1 = JFRST(N) + NJ(N)-1
         JN = JFRST(N)
         JINC = -1
        ENDIF
C
C
C--- Bound vortex legs plotted
        IF(LBOUNDLEG) THEN
         IP = 0
         DO J = J1, JN, JINC
           I1 = IJFRST(J)
           NV = NVSTRP(J)
           DO II = 1, NV
             IV = I1 + II-1
             IP = IP+1
             XAVE = 0.5*(RV1(1,IV) + RV2(1,IV))
             YAVE = 0.5*(RV1(2,IV) + RV2(2,IV))
             ZAVE = 0.5*(RV1(3,IV) + RV2(3,IV))
C
C--- Optionally regress vortex edges slightly for visibility
             REGR = 0.0
             PTS_LINES(1,1,IP) = (1.0-REGR)*(RV1(1,IV) + REGR*XAVE)
             PTS_LINES(2,1,IP) = (1.0-REGR)*(RV1(2,IV) + REGR*YAVE)
             PTS_LINES(3,1,IP) = (1.0-REGR)*(RV1(3,IV) + REGR*ZAVE)
             PTS_LINES(1,2,IP) = (1.0-REGR)*(RV2(1,IV) + REGR*XAVE)
             PTS_LINES(2,2,IP) = (1.0-REGR)*(RV2(2,IV) + REGR*YAVE)
             PTS_LINES(3,2,IP) = (1.0-REGR)*(RV2(3,IV) + REGR*ZAVE)
             ID_LINES(IP) = J
           END DO
         END DO
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C 
         CALL GETCOLOR(ICOL)
         CALL NEWCOLORNAME('MAGENTA')
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         CALL NEWCOLOR(ICOL)
        ENDIF
C
C--- Control points plotted
        IF(LCNTLPTS) THEN
         SCL = 0.01*MIN(CREF,0.5*BREF)
C
         IP = 0
         DO J = J1, JN, JINC
           I1 = IJFRST(J)
           NV = NVSTRP(J)
C--- Define vector on strip plane for control point "x" mark
           SVEC1(1) =  1.0    *SCL
           SVEC1(2) = -ENSZ(J)*SCL
           SVEC1(3) =  ENSY(J)*SCL
           SVEC2(1) = -SVEC1(1)
           SVEC2(2) =  SVEC1(2)
           SVEC2(3) =  SVEC1(3)
           DO II = 1, NV
             IV = I1 + II-1
             IP = IP+1
             PTS_LINES(1,1,IP) = RC(1,IV) - SVEC1(1)  
             PTS_LINES(2,1,IP) = RC(2,IV) - SVEC1(2)  
             PTS_LINES(3,1,IP) = RC(3,IV) - SVEC1(3)  
             PTS_LINES(1,2,IP) = RC(1,IV) + SVEC1(1)  
             PTS_LINES(2,2,IP) = RC(2,IV) + SVEC1(2)  
             PTS_LINES(3,2,IP) = RC(3,IV) + SVEC1(3)  
             ID_LINES(IP) = J
             IP = IP+1
             PTS_LINES(1,1,IP) = RC(1,IV) - SVEC2(1)  
             PTS_LINES(2,1,IP) = RC(2,IV) - SVEC2(2)  
             PTS_LINES(3,1,IP) = RC(3,IV) - SVEC2(3)  
             PTS_LINES(1,2,IP) = RC(1,IV) + SVEC2(1)  
             PTS_LINES(2,2,IP) = RC(2,IV) + SVEC2(2)  
             PTS_LINES(3,2,IP) = RC(3,IV) + SVEC2(3)  
             ID_LINES(IP) = J
           END DO
         END DO
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C 
         CALL GETCOLOR(ICOL)
         CALL NEWCOLORNAME('YELLOW')
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         CALL NEWCOLOR(ICOL)
C
C
         IP = 0
         DO J = J1, JN, JINC
           I1 = IJFRST(J)
           NV = NVSTRP(J)
C--- Define vector on strip plane for control point "x" mark
           SVEC1(1) =  1.0    *SCL
           SVEC1(2) = -ENSZ(J)*SCL
           SVEC1(3) =  ENSY(J)*SCL
           SVEC2(1) = -SVEC1(1)
           SVEC2(2) =  SVEC1(2)
           SVEC2(3) =  SVEC1(3)
           DO II = 1, NV
             IV = I1 + II-1
cc             IF(LVNC(IV)) THEN
              IP = IP+1
              PTS_LINES(1,1,IP) = RV(1,IV) - SVEC1(1)  
              PTS_LINES(2,1,IP) = RV(2,IV) - SVEC1(2)  
              PTS_LINES(3,1,IP) = RV(3,IV) - SVEC1(3)  
              PTS_LINES(1,2,IP) = RV(1,IV) + SVEC1(1)  
              PTS_LINES(2,2,IP) = RV(2,IV) + SVEC1(2)  
              PTS_LINES(3,2,IP) = RV(3,IV) + SVEC1(3)  
              ID_LINES(IP) = J
              IP = IP+1
              PTS_LINES(1,1,IP) = RV(1,IV) - SVEC2(1)  
              PTS_LINES(2,1,IP) = RV(2,IV) - SVEC2(2)  
              PTS_LINES(3,1,IP) = RV(3,IV) - SVEC2(3)  
              PTS_LINES(1,2,IP) = RV(1,IV) + SVEC2(1)  
              PTS_LINES(2,2,IP) = RV(2,IV) + SVEC2(2)  
              PTS_LINES(3,2,IP) = RV(3,IV) + SVEC2(3)  
              ID_LINES(IP) = J
cc             ENDIF
           END DO
         END DO
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C 
         CALL GETCOLOR(ICOL)
         CALL NEWCOLORNAME('ORANGE')
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         CALL NEWCOLOR(ICOL)

        ENDIF
C
C
C--- Chordlines plotted
        IF(LCHORDLINE) THEN
         IP = 0
         DO J = J1, JN, JINC
           IP = IP+1
           PTS_LINES(1,1,IP) = RLE1(1,J)
           PTS_LINES(2,1,IP) = RLE1(2,J)
           PTS_LINES(3,1,IP) = RLE1(3,J)
           PTS_LINES(1,2,IP) = RLE1(1,J) + CHORD1(J)
           PTS_LINES(2,2,IP) = RLE1(2,J)
           PTS_LINES(3,2,IP) = RLE1(3,J)
           ID_LINES(IP) = J
C--- Plot trailing legs back 2.0*BREF if enabled
           IF(LWAKEPLT .AND. LFWAKE(N)) THEN
             IP = IP+1
             PTS_LINES(1,1,IP) = PTS_LINES(1,2,IP-1)
             PTS_LINES(2,1,IP) = PTS_LINES(2,2,IP-1)
             PTS_LINES(3,1,IP) = PTS_LINES(3,2,IP-1)
             PTS_LINES(1,2,IP) = PTS_LINES(1,1,IP) + 2.0*BREF
             PTS_LINES(2,2,IP) = PTS_LINES(2,1,IP)
             PTS_LINES(3,2,IP) = PTS_LINES(3,1,IP)
             ID_LINES(IP) = 0
           ENDIF
         END DO
         IP = IP+1
         PTS_LINES(1,1,IP) = RLE2(1,JN)
         PTS_LINES(2,1,IP) = RLE2(2,JN)
         PTS_LINES(3,1,IP) = RLE2(3,JN)
         PTS_LINES(1,2,IP) = RLE2(1,JN) + CHORD2(JN)
         PTS_LINES(2,2,IP) = RLE2(2,JN)
         PTS_LINES(3,2,IP) = RLE2(3,JN)
         ID_LINES(IP) = JN
C--- Plot trailing legs back 2.0*BREF if enabled
         IF(LWAKEPLT .AND. LFWAKE(N)) THEN
           IP = IP+1
           PTS_LINES(1,1,IP) = PTS_LINES(1,2,IP-1)
           PTS_LINES(2,1,IP) = PTS_LINES(2,2,IP-1)
           PTS_LINES(3,1,IP) = PTS_LINES(3,2,IP-1)
           PTS_LINES(1,2,IP) = PTS_LINES(1,1,IP) + 2.0*BREF
           PTS_LINES(2,2,IP) = PTS_LINES(2,1,IP)
           PTS_LINES(3,2,IP) = PTS_LINES(3,1,IP)
           ID_LINES(IP) = 0
         ENDIF
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C--- plot chordlines dashed 
         CALL NEWPAT(lmask2)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         CALL NEWPAT(lmask0)
        ENDIF
C
C--- Strip camberline plotted (approx., integrated from camber slopes)
        IF(LCAMBER) THEN
         CALL GETCOLOR(ICOL)
         DO IPASS = 1, 2
           IP = 0
           DO J = J1, JN, JINC
             I1 = IJFRST(J)
             NV = NVSTRP(J)
             XX0 = RLE(1,J)
             YY0 = RLE(2,J)
             ZZ0 = RLE(3,J) + EPS
             DO II = 1, NV
               IV = I1 + II-1
C
               DENT = 0.
               IF(IPASS.EQ.1) THEN
                DO K = 1, NCONTROL
                  DENT = DENT
     &               + 0.5*(ENC_D(1,IV,K) + ENV_D(1,IV,K))*DELCON(K)
                ENDDO
               ENDIF
               ENTOT(1) = 0.5*(ENC(1,IV) + ENV(1,IV)) + DENT
C
               TC = -DXV(IV)*ENTOT(1)
               XX1 = XX0 + DXV(IV)
               YY1 = YY0 + TC*ENSY(J)
               ZZ1 = ZZ0 + TC*ENSZ(J)
C
               IP = IP+1
               PTS_LINES(1,1,IP) = XX0
               PTS_LINES(2,1,IP) = YY0
               PTS_LINES(3,1,IP) = ZZ0
               PTS_LINES(1,2,IP) = XX1
               PTS_LINES(2,2,IP) = YY1
               PTS_LINES(3,2,IP) = ZZ1
               ID_LINES(IP) = 0
C
               XX0 = XX1
               YY0 = YY1
               ZZ0 = ZZ1
             ENDDO
           ENDDO
C
           NLINES = IP
           NPROJ = 2*NLINES
           CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C 
           IF(IPASS.EQ.1) THEN
            CALL NEWCOLORNAME('ORANGE')
           ELSE
            CALL NEWCOLORNAME('CYAN')
           ENDIF
           CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         ENDDO
         CALL NEWCOLOR(ICOL)
        ENDIF
C
C--- Strip loading plotted
        IF(LLOADPLT) THEN
C 
         CALL GETPEN(IPN)
         CALL GETCOLOR(ICOL)
C
         CALL NEWCOLORNAME('GREEN')
C
C--- Plot the load vectors (up from surface)
         CALL NEWPEN(5)
         IP = 0
         CPSCL = CPFAC*CREF
         DO J = J1, JN, JINC
           I1 = IJFRST(J)
           NV = NVSTRP(J)
           DO II = 1, NV
             IV = I1 + II-1
             IP = IP+1
             XAVE = RV(1,IV)
             YAVE = RV(2,IV)
             ZAVE = RV(3,IV)
             DELYZ = DCP(IV) * CPSCL
             XLOAD = XAVE
             YLOAD = YAVE + DELYZ*ENSY(J) 
             ZLOAD = ZAVE + DELYZ*ENSZ(J)

c             XLOAD = XAVE + DELYZ*ENC(1,IV)
c             YLOAD = YAVE + DELYZ*ENC(2,IV)
c             ZLOAD = ZAVE + DELYZ*ENC(3,IV)
C
             PTS_LINES(1,1,IP) = XAVE
             PTS_LINES(2,1,IP) = YAVE
             PTS_LINES(3,1,IP) = ZAVE
             PTS_LINES(1,2,IP) = XLOAD
             PTS_LINES(2,2,IP) = YLOAD
             PTS_LINES(3,2,IP) = ZLOAD
             ID_LINES(IP) = 0
           END DO
         END DO
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C
C--- Plot the loading lines 
         CALL NEWPEN(2)
         CALL NEWCOLORNAME('RED')
         IP = 0
         DO J = J1, JN, JINC
           I1 = IJFRST(J)
           NV = NVSTRP(J)
           DO II = 1, NV
             IV = I1 + II-1
             XAVE = RV(1,IV)
             YAVE = RV(2,IV)
             ZAVE = RV(3,IV)
             DELYZ = DCP(IV) * CPSCL
             XLOAD = XAVE
             YLOAD = YAVE + DELYZ*ENSY(J) 
             ZLOAD = ZAVE + DELYZ*ENSZ(J)

c             XLOAD = XAVE + DELYZ*ENC(1,IV) 
c             YLOAD = YAVE + DELYZ*ENC(2,IV) 
c             ZLOAD = ZAVE + DELYZ*ENC(3,IV)

             IF(II.GT.1) THEN
               IP = IP+1
               PTS_LINES(1,1,IP) = XLOADOLD
               PTS_LINES(2,1,IP) = YLOADOLD
               PTS_LINES(3,1,IP) = ZLOADOLD
               PTS_LINES(1,2,IP) = XLOAD
               PTS_LINES(2,2,IP) = YLOAD
               PTS_LINES(3,2,IP) = ZLOAD
               ID_LINES(IP) = 0
             ENDIF             
             XLOADOLD = XLOAD
             YLOADOLD = YLOAD
             ZLOADOLD = ZLOAD
           END DO
         END DO
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
         CALL NEWCOLOR(ICOL)
         CALL NEWPEN(IPN)
        ENDIF
C
C
C--- Surface boundary lines plotted
        IP = 0
        IP = IP+1
        PTS_LINES(1,1,IP) = RLE1(1,J1)
        PTS_LINES(2,1,IP) = RLE1(2,J1)
        PTS_LINES(3,1,IP) = RLE1(3,J1)
        PTS_LINES(1,2,IP) = RLE1(1,J1) + CHORD1(J1)
        PTS_LINES(2,2,IP) = RLE1(2,J1)
        PTS_LINES(3,2,IP) = RLE1(3,J1)
        ID_LINES(IP) = J1
C--- Plot trailing legs back 2.0*BREF if enabled and exist
        IF(LWAKEPLT .AND. LFWAKE(N)) THEN
          IP = IP+1
          PTS_LINES(1,1,IP) = PTS_LINES(1,2,IP-1)
          PTS_LINES(2,1,IP) = PTS_LINES(2,2,IP-1)
          PTS_LINES(3,1,IP) = PTS_LINES(3,2,IP-1)
          PTS_LINES(1,2,IP) = PTS_LINES(1,1,IP) + 2.0*BREF
          PTS_LINES(2,2,IP) = PTS_LINES(2,1,IP)
          PTS_LINES(3,2,IP) = PTS_LINES(3,1,IP)
          ID_LINES(IP) = 0
        ENDIF
C
        IP = IP+1
        PTS_LINES(1,1,IP) = RLE2(1,JN)
        PTS_LINES(2,1,IP) = RLE2(2,JN)
        PTS_LINES(3,1,IP) = RLE2(3,JN)
        PTS_LINES(1,2,IP) = RLE2(1,JN) + CHORD2(JN)
        PTS_LINES(2,2,IP) = RLE2(2,JN)
        PTS_LINES(3,2,IP) = RLE2(3,JN)
        ID_LINES(IP) = JN
C--- Plot trailing legs back 2.0*BREF if enabled and exist
        IF(LWAKEPLT .AND. LFWAKE(N)) THEN
          IP = IP+1
          PTS_LINES(1,1,IP) = PTS_LINES(1,2,IP-1)
          PTS_LINES(2,1,IP) = PTS_LINES(2,2,IP-1)
          PTS_LINES(3,1,IP) = PTS_LINES(3,2,IP-1)
          PTS_LINES(1,2,IP) = PTS_LINES(1,1,IP) + 2.0*BREF
          PTS_LINES(2,2,IP) = PTS_LINES(2,1,IP)
          PTS_LINES(3,2,IP) = PTS_LINES(3,1,IP)
          ID_LINES(IP) = 0
        ENDIF
C
        DO J = J1, JN, JINC
          IP = IP+1
          PTS_LINES(1,1,IP) = RLE1(1,J)
          PTS_LINES(2,1,IP) = RLE1(2,J)
          PTS_LINES(3,1,IP) = RLE1(3,J)
          PTS_LINES(1,2,IP) = RLE2(1,J)
          PTS_LINES(2,2,IP) = RLE2(2,J)
          PTS_LINES(3,2,IP) = RLE2(3,J)
          ID_LINES(IP) = J
          IP = IP+1
          PTS_LINES(1,1,IP) = RLE1(1,J) + CHORD1(J)
          PTS_LINES(2,1,IP) = RLE1(2,J)
          PTS_LINES(3,1,IP) = RLE1(3,J)
          PTS_LINES(1,2,IP) = RLE2(1,J) + CHORD2(J)
          PTS_LINES(2,2,IP) = RLE2(2,J)
          PTS_LINES(3,2,IP) = RLE2(3,J)
          ID_LINES(IP) = J
        END DO
C
        NLINES = IP
        NPROJ = 2*NLINES
        CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
        CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C
C
C*************************************************************************
C---Now do any labels (integer indices for surfaces,strips or vortices)
C
C---Surface labels
        IF(LABEL_SURF) THEN
          JMID = JFRST(N) + IFIX(0.5*FLOAT(NJ(N))) - 1
          PLAB(1) = RLE(1,JMID) + 0.6*CHORD(JMID)
          PLAB(2) = RLE(2,JMID)
          PLAB(3) = RLE(3,JMID)
          CALL VIEWPROJ(PLAB,1,PTS_LPROJ)
          ID = JMID
          LVIS=.TRUE.
          IF(LHID) THEN
            NGRP = 0
            IDUM(1) = 0
            DUM(1,1) = 0.
            CALL HIDPNT(PTS_LPROJ,ID,NGRP,IDUM,DUM,NTRI,TRI,LVIS)
          ENDIF
          ILAB = N
          IF (LVIS) CALL PLTINT(PTS_LPROJ,ILAB,1.5*CHSIZE,.FALSE.)
        ENDIF
C
C---Strip labels
        IF(LABEL_STRP) THEN
          NGRP = 0
          IDUM(1) = 0
          DUM(1,1) = 0.
          CALL GETPEN(IPN)
          CALL NEWPEN(1)
          CALL GETCOLOR(ICOL)
          CALL NEWCOLORNAME('CYAN')
          DO J = J1, JN, JINC
            PLAB(1) = RLE(1,J) + 0.4*CHORD(J)
            PLAB(2) = RLE(2,J)
            PLAB(3) = RLE(3,J)
            CALL VIEWPROJ(PLAB,1,PTS_LPROJ)
            ID = J
            LVIS=.TRUE.
            IF(LHID) THEN
              CALL HIDPNT(PTS_LPROJ,ID,NGRP,IDUM,DUM,NTRI,TRI,LVIS)
            ENDIF
            ILAB = J
            IF(LVIS) CALL PLTINT(PTS_LPROJ,ILAB,1.2*CHSIZE,.FALSE.)
          END DO
          CALL NEWPEN(IPN)
          CALL NEWCOLOR(ICOL)
        ENDIF
C
C---Vortex labels
        IF(LABEL_VRTX) THEN
          NGRP = 0
          IDUM(1) = 0
          DUM(1,1) = 0.
          CALL GETPEN(IPN)
          CALL NEWPEN(1)
          CALL GETCOLOR(ICOL)
          CALL NEWCOLORNAME('RED')
          DO J = J1, JN, JINC
            I1 = IJFRST(J)
            NV = NVSTRP(J)
            DO II = 1, NV
              IV = I1 + II-1
              IP = IP+1
              XAVE = RV(1,IV)
              YAVE = RV(2,IV)
              ZAVE = RV(3,IV)
              PLAB(1) = 0.5*(XAVE+RC(1,IV))
              PLAB(2) = 0.5*(YAVE+RC(2,IV))
              PLAB(3) = 0.5*(ZAVE+RC(3,IV))
              CALL VIEWPROJ(PLAB,1,PTS_LPROJ)
              ID = J
              LVIS=.TRUE.
              IF(LHID) THEN
                CALL HIDPNT(PTS_LPROJ,ID,NGRP,IDUM,DUM,NTRI,TRI,LVIS)
              ENDIF
              ILAB = IV
              IF(LVIS) CALL PLTINT(PTS_LPROJ,ILAB,CHSIZE,.FALSE.)
            END DO
          END DO
          CALL NEWPEN(IPN)
          CALL NEWCOLOR(ICOL)
        ENDIF
C
 100  CONTINUE
C

      IF(LNRMLPLT) THEN
      DO IPASS = 1, 2
C
      CALL GETCOLOR(ICOL)
      IF(IPASS.EQ.1) THEN
C----- with control deflections
       CALL NEWCOLORNAME('ORANGE')
      ELSE
C----- without control deflections
       CALL NEWCOLORNAME('BLUE')
      ENDIF
C
C---- go over surfaces again
      DO 200 N = 1, NSURF
C
C------ do only surfaces which are to be plotted
        IF(.NOT.LPLTSURF(N)) GO TO 200
C
        J1 = JFRST(N)
        JN = JFRST(N) + NJ(N)-1
        JINC = 1
        IF(IMAGS(N).LT.0) THEN
         J1 = JFRST(N) + NJ(N)-1
         JN = JFRST(N)
         JINC = -1
        ENDIF
C
C----- Normal vectors plotted
        SCL = ENFAC*CREF
C
        IP = 0
        DO J = J1, JN, JINC
          I1 = IJFRST(J)
          NV = NVSTRP(J)
          DO II = 1, NV
            IV = I1 + II-1
C
            ENTOT(1) = ENC(1,IV)
            ENTOT(2) = ENC(2,IV)
            ENTOT(3) = ENC(3,IV)
            IF(IPASS.EQ.1) THEN
             DO K = 1, NCONTROL
               ENTOT(1) = ENTOT(1) + ENC_D(1,IV,K)*DELCON(K)
               ENTOT(2) = ENTOT(2) + ENC_D(2,IV,K)*DELCON(K)
               ENTOT(3) = ENTOT(3) + ENC_D(3,IV,K)*DELCON(K)
             ENDDO
            ENDIF
C
C-- Define vector from control point in direction of normal vector 
            SVEC1(1) = ENTOT(1) * SCL
            SVEC1(2) = ENTOT(2) * SCL
            SVEC1(3) = ENTOT(3) * SCL
            SVEC2(1) = 0.02*SCL
            SVEC2(2) = 0.0
            SVEC2(3) = 0.0
C-- Vector out from control point
            IP = IP+1
            PTS_LINES(1,1,IP) = RC(1,IV)
            PTS_LINES(2,1,IP) = RC(2,IV)
            PTS_LINES(3,1,IP) = RC(3,IV)
            PTS_LINES(1,2,IP) = RC(1,IV) + SVEC1(1)  
            PTS_LINES(2,2,IP) = RC(2,IV) + SVEC1(2)  
            PTS_LINES(3,2,IP) = RC(3,IV) + SVEC1(3)  
            ID_LINES(IP) = 0
C-- Arrow head for normal vector
            IP = IP+1
            PTS_LINES(1,1,IP) = RC(1,IV) + SVEC1(1)  
            PTS_LINES(2,1,IP) = RC(2,IV) + SVEC1(2)  
            PTS_LINES(3,1,IP) = RC(3,IV) + SVEC1(3)  
            PTS_LINES(1,2,IP) = RC(1,IV) + 0.8*SVEC1(1) + SVEC2(1) 
            PTS_LINES(2,2,IP) = RC(2,IV) + 0.8*SVEC1(2) + SVEC2(2) 
            PTS_LINES(3,2,IP) = RC(3,IV) + 0.8*SVEC1(3) + SVEC2(3)  
            ID_LINES(IP) = 0
            IP = IP+1
            PTS_LINES(1,1,IP) = RC(1,IV) + SVEC1(1)  
            PTS_LINES(2,1,IP) = RC(2,IV) + SVEC1(2)  
            PTS_LINES(3,1,IP) = RC(3,IV) + SVEC1(3)  
            PTS_LINES(1,2,IP) = RC(1,IV) + 0.8*SVEC1(1) - SVEC2(1) 
            PTS_LINES(2,2,IP) = RC(2,IV) + 0.8*SVEC1(2) - SVEC2(2) 
            PTS_LINES(3,2,IP) = RC(3,IV) + 0.8*SVEC1(3) - SVEC2(3)  
            ID_LINES(IP) = 0
          END DO
        END DO
        NLINES = IP
        NPROJ = 2*NLINES
        CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
        CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
 200  CONTINUE
C
      CALL NEWCOLOR(ICOL)
      ENDDO
      ENDIF
C
C
C---- go over bodies
      DO 300 N = 1, NBODY
C
C------ do only bodies which are to be plotted
        IF(.NOT.LPLTBODY(N)) GO TO 300
C
        L1 = LFRST(N)
        LN = LFRST(N) + NL(N)-1
        LINC = 1
C
C------ source lines plotted
        IP = 0
        DO L = L1, LN-LINC, LINC
          IP = IP+1
          PTS_LINES(1,1,IP) = RL(1,L)
          PTS_LINES(2,1,IP) = RL(2,L)
          PTS_LINES(3,1,IP) = RL(3,L)
          PTS_LINES(1,2,IP) = RL(1,L+LINC)
          PTS_LINES(2,2,IP) = RL(2,L+LINC)
          PTS_LINES(3,2,IP) = RL(3,L+LINC)
          ID_LINES(IP) = 0
        END DO
C
        NLINES = IP
        NPROJ = 2*NLINES
        CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
C
        CALL GETCOLOR(ICOL)
        CALL NEWCOLORNAME('MAGENTA')
        CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
        CALL NEWCOLOR(ICOL)
C
C
C------ Chordlines plotted
ccc        IF(LCHORDLINE) THEN
         IP = 0
         DO L = L1, LN, LINC
C--------- set "horizontal" and "vertical" body section unit vectors
           IF(LINC.GT.0) THEN
            LM = MAX(L-1,L1)
            LP = MIN(L+1,LN)
           ELSE
            LM = MAX(L-1,LN)
            LP = MIN(L+1,L1)
           ENDIF
           DXL = RL(1,LP) - RL(1,LM)
           DYL = RL(2,LP) - RL(2,LM)
           DZL = RL(3,LP) - RL(3,LM)
           DSL = SQRT(DXL**2 + DYL**2 + DZL**2)
           IF(DSL.NE.0.0) THEN
            DXL = DXL/DSL
            DYL = DYL/DSL
            DZL = DZL/DSL
           ENDIF
C     
           UHX = -DYL
           UHY =  DXL
           UHZ = 0.
C
           UVX = DYL*UHZ - DZL*UHY
           UVY = DZL*UHX - DXL*UHZ
           UVZ = DXL*UHY - DYL*UHX
C
           DO K = 1, KK
             THK1 = 2.0*PI*FLOAT(K-1)/FLOAT(KK)
             THK2 = 2.0*PI*FLOAT(K  )/FLOAT(KK)
C
             HX1 = UHX*COS(THK1)
             HY1 = UHY*COS(THK1)
             HZ1 = UHZ*COS(THK1)
C
             VX1 = UVX*SIN(THK1)
             VY1 = UVY*SIN(THK1)
             VZ1 = UVZ*SIN(THK1)
C
             HX2 = UHX*COS(THK2)
             HY2 = UHY*COS(THK2)
             HZ2 = UHZ*COS(THK2)
C
             VX2 = UVX*SIN(THK2)
             VY2 = UVY*SIN(THK2)
             VZ2 = UVZ*SIN(THK2)
C
             IP = IP+1
             PTS_LINES(1,1,IP) = RL(1,L) + HX1*RADL(L) + VX1*RADL(L)
             PTS_LINES(2,1,IP) = RL(2,L) + HY1*RADL(L) + VY1*RADL(L)
             PTS_LINES(3,1,IP) = RL(3,L) + HZ1*RADL(L) + VZ1*RADL(L)
             PTS_LINES(1,2,IP) = RL(1,L) + HX2*RADL(L) + VX2*RADL(L)
             PTS_LINES(2,2,IP) = RL(2,L) + HY2*RADL(L) + VY2*RADL(L)
             PTS_LINES(3,2,IP) = RL(3,L) + HZ2*RADL(L) + VZ2*RADL(L)
             ID_LINES(IP) = 0
           ENDDO
         END DO
C
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
ccc        ENDIF
C
C------ source line loading plotted
        IF(LLOADPLT) THEN
C 
         CALL GETPEN(IPN)
         CALL GETCOLOR(ICOL)
C
         CALL NEWCOLORNAME('GREEN')
C
C--- Plot the load vectors (up from surface)
         CALL NEWPEN(5)
         IP = 0
         CPSCL = CPFAC*CREF
         DO L = L1, LN
           IP = IP+1
           XAVE = 0.5*(RL(1,L) + RL(1,L+1))
           YAVE = 0.5*(RL(2,L) + RL(2,L+1))
           ZAVE = 0.5*(RL(3,L) + RL(3,L+1))
           XLOAD = XAVE + DCPB(1,L) * CPSCL
           YLOAD = YAVE + DCPB(2,L) * CPSCL
           ZLOAD = ZAVE + DCPB(3,L) * CPSCL
C
           PTS_LINES(1,1,IP) = XAVE
           PTS_LINES(2,1,IP) = YAVE
           PTS_LINES(3,1,IP) = ZAVE
           PTS_LINES(1,2,IP) = XLOAD
           PTS_LINES(2,2,IP) = YLOAD
           PTS_LINES(3,2,IP) = ZLOAD
           ID_LINES(IP) = 0
         END DO
         NLINES = IP
         NPROJ = 2*NLINES
         CALL VIEWPROJ(PTS_LINES,NPROJ,PTS_LPROJ)
         CALL PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C---Bugfix 8/8/08 HHY
         CALL NEWPEN(IPN)
         CALL NEWCOLOR(ICOL)
C
        ENDIF
C
C
C*************************************************************************
C---Now do any labels (integer indices for surfaces,strips or vortices)
C
C--- Body labels
        IF(LABEL_BODY) THEN
          JMID = JFRST(N) + IFIX(0.5*FLOAT(NJ(N))) - 1
          PLAB(1) = RLE(1,JMID) + 0.6*CHORD(JMID)
          PLAB(2) = RLE(2,JMID)
          PLAB(3) = RLE(3,JMID)
          CALL VIEWPROJ(PLAB,1,PTS_LPROJ)
          ID = JMID
          LVIS=.TRUE.
          IF(LHID) THEN
            NGRP = 0
            IDUM(1) = 0
            DUM(1,1) = 0.
            CALL HIDPNT(PTS_LPROJ,ID,NGRP,IDUM,DUM,NTRI,TRI,LVIS)
          ENDIF
          ILAB = N
          IF (LVIS) CALL PLTINT(PTS_LPROJ,ILAB,1.5*CHSIZE,.FALSE.)
        ENDIF
C
 300  CONTINUE
C
      RETURN
      END ! PLOTGEOM


      SUBROUTINE PLOTLINES(NLINES,PTS_LPROJ,ID_LINES)
C...PURPOSE:    Plot line segments with hidden line 
C               routine.
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
      REAL PTS_LPROJ(3,2,*), ID_LINES(*)
C
      PARAMETER (NVX=32)
      REAL ALF(2,NVX)
C
      INTEGER IDUM(1)
      REAL DUM(1,1)
C
      ALF(1,1) = 0.0
      ALF(2,1) = 1.0
      NALF = 1
C
      DO L=1, NLINES
C-----  Check for hidden line treatment
        IF(LHID) THEN
          NALF = NVX
          NGRP = 0
          ID = ID_LINES(L)
          IDUM(1) = 0
          DUM(1,1) = 0.0
          CALL HIDLIN(PTS_LPROJ(1,1,L),ID,NGRP,IDUM,DUM,
     &                NTRI,TRI,NALF,ALF)
        ENDIF

C-----  If any of the line segment is visible, plot the visible pieces
        IF(NALF.GT.0) CALL PLTSEG(PTS_LPROJ(1,1,L),ALF,NALF)
      END DO
C
      RETURN
      END ! PLOTLINES
