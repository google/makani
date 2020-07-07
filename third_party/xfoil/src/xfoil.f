C***********************************************************************
C    Module:  xfoil.f
C 
C    Copyright (C) 2000 Mark Drela 
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
      PROGRAM XFOIL
C--- Uncomment for Win32/Compaq Visual Fortran compiler (needed for GETARG)
ccc      USE DFLIB
C
      INCLUDE 'XFOIL.INC'
      CHARACTER*4 COMAND
      CHARACTER*128 COMARG, PROMPT
      CHARACTER*1 ANS
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
C---- max panel angle threshold for warning
      DATA ANGTOL / 40.0 /
C
      VERSION = 6.99
      WRITE(*,1005) VERSION
 1005 FORMAT(
     &  /' ==================================================='
     &  /'  XFOIL Version', F5.2
     &  /'  Copyright (C) 2000   Mark Drela, Harold Youngren'
     & //'  This software comes with ABSOLUTELY NO WARRANTY,' 
     &  /'    subject to the GNU General Public License.'
     & //'  Caveat computor'
     &  /' ===================================================')
C
      CALL INIT
      LU = 8
      CALL GETDEF(LU,'xfoil.def', .TRUE.)
C
C---- try to read airfoil from command line argument, if any
      FNAME = ' '
      NARG = IARGC()
      IF(NARG.GT.0) CALL GETARG(NARG,FNAME)
C
      IF(FNAME(1:1) .NE. ' ') THEN
       CALL LOAD(FNAME,ITYPE)
C
       IF(ITYPE.GT.0 .AND. NB.GT.0) THEN
ccc     CALL PANGEN(.TRUE.)
        CALL ABCOPY(.TRUE.)
C
        CALL CANG(X,Y,N,0, IMAX,AMAX)
        IF(ABS(AMAX).GT.ANGTOL) THEN
         WRITE(*,1081) AMAX, IMAX
 1081    FORMAT(
     &  /' WARNING: Poor input coordinate distribution'
     &  /'          Excessive panel angle', F7.1,'  at i =', I4
     &  /'          Repaneling with PANE and/or PPAR suggested'
     &  /'           (doing GDES,CADD before repaneling _may_'
     &  /'            improve excessively coarse LE spacing' )
         CALL PANPLT
        ENDIF
       ENDIF
      ENDIF
C
      WRITE(*,1100) XCMREF,YCMREF,NPAN
 1100 FORMAT(
     &  /'   QUIT    Exit program'
     & //'  .OPER    Direct operating point(s)'
     &  /'  .MDES    Complex mapping design routine'
     &  /'  .QDES    Surface speed design routine'
     &  /'  .GDES    Geometry design routine'
     & //'   SAVE f  Write airfoil to labeled coordinate file'
     &  /'   PSAV f  Write airfoil to plain coordinate file'
     &  /'   ISAV f  Write airfoil to ISES coordinate file'
     &  /'   MSAV f  Write airfoil to MSES coordinate file'
     &  /'   REVE    Reverse written-airfoil node ordering'
     &  /'   DELI i  Change written-airfoil file delimiters'
     & //'   LOAD f  Read buffer airfoil from coordinate file'
     &  /'   NACA i  Set NACA 4,5-digit airfoil and buffer airfoil'
     &  /'   INTE    Set buffer airfoil by interpolating two airfoils'
     &  /'   NORM    Buffer airfoil normalization toggle'
     &  /'   HALF    Halve the number of points in buffer airfoil'
     &  /'   XYCM rr Change CM reference location, currently ',2F8.5
     & //'   BEND    Display structural properties of current airfoil' 
     & //'   PCOP    Set current-airfoil panel nodes directly',
     &                ' from buffer airfoil points'
     &  /'   PANE    Set current-airfoil panel nodes (',I4,' )',
     &                ' based on curvature'
     &  /'  .PPAR    Show/change paneling'
     & //'  .PLOP    Plotting options'
     & //'   WDEF f  Write  current-settings file'
     &  /'   RDEF f  Reread current-settings file'
     &  /'   NAME s  Specify new airfoil name'
     &  /'   NINC    Increment name version number'
     & //'   Z       Zoom    | (available in all menus)'
     &  /'   U       Unzoom  | ')
C
C---- start of menu loop
  500 CONTINUE
      CALL ASKC(' XFOIL^',COMAND,COMARG)
C
C---- get command line numeric arguments, if any
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
C===============================================
      IF(COMAND.EQ.'    ') THEN
       GO TO 500
C
C===============================================
      ELSEIF(COMAND.EQ.'?   ') THEN
       WRITE(*,1100) XCMREF, YCMREF, NPAN
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
      ELSEIF(COMAND.EQ.'MDES') THEN
       CALL MDES
C
C===============================================
      ELSEIF(COMAND.EQ.'QDES') THEN
       CALL QDES
C
C===============================================
      ELSEIF(COMAND.EQ.'GDES') THEN
       CALL GDES
C
C===============================================
      ELSEIF(COMAND.EQ.'SAVE') THEN
       CALL SAVE(1,COMARG)
C
C===============================================
      ELSEIF(COMAND.EQ.'PSAV') THEN
       CALL SAVE(0,COMARG)
C
C===============================================
      ELSEIF(COMAND.EQ.'USAV') THEN
       CALL SAVE(-1,COMARG)
C
C===============================================
      ELSEIF(COMAND.EQ.'ISAV') THEN
       CALL SAVE(2,COMARG)
C
C===============================================
      ELSEIF(COMAND.EQ.'MSAV') THEN
       CALL MSAVE(COMARG)
C
C===============================================
      ELSEIF(COMAND.EQ.'REVE') THEN
       LCLOCK = .NOT.LCLOCK
       IF(LCLOCK) THEN
        WRITE(*,*) 'Airfoil will be written in clockwise order'
       ELSE
        WRITE(*,*) 'Airfoil will be written in counterclockwise order'
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'DELI') THEN
   40  CONTINUE
       IF(NINPUT.GE.1) THEN
        KDNEW = IINPUT(1)
       ELSE
        WRITE(*,2100) KDELIM
 2100   FORMAT(/'  --------------------------'
     &         /'   0  blank'
     &         /'   1  comma'
     &         /'   2  tab',
     &        //'  currently, delimiter =', I2)
        CALL ASKI('Enter new delimiter',KDNEW)
       ENDIF
C
       IF(KDNEW.LT.0 .OR. KDNEW.GT.2) THEN
        NINPUT = 0
        GO TO 40
       ELSE
        KDELIM = KDNEW
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'LOAD') THEN
       CALL LOAD(COMARG,ITYPE)
       IF(ITYPE.GT.0 .AND. NB.GT.0) THEN
ccc       CALL PANGEN(.TRUE.)
        CALL ABCOPY(.TRUE.)
C
        CALL CANG(X,Y,N,0, IMAX,AMAX)
        IF(ABS(AMAX).GT.ANGTOL) THEN
         WRITE(*,1081) AMAX, IMAX
         CALL PANPLT
        ENDIF
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'NACA') THEN
       CALL NACA(IINPUT(1))
C
C===============================================
      ELSEIF(COMAND.EQ.'INTE') THEN
       CALL INTE
C
C===============================================
      ELSEIF(COMAND.EQ.'INTX') THEN
       CALL INTX
C
C===============================================
      ELSEIF(COMAND.EQ.'NORM') THEN
       LNORM = .NOT.LNORM
       IF(LNORM) THEN
        WRITE(*,*) 'Loaded airfoil will  be normalized'
       ELSE
        WRITE(*,*) 'Loaded airfoil won''t be normalized'
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'HALF') THEN
       CALL HALF(XB,YB,SB,NB)
       CALL SCALC(XB,YB,SB,NB)
       CALL SEGSPL(XB,XBP,SB,NB)
       CALL SEGSPL(YB,YBP,SB,NB)
C
       CALL GEOPAR(XB,XBP,YB,YBP,SB,NB, W1,
     &             SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &             EI11BA,EI22BA,APX1BA,APX2BA,
     &             EI11BT,EI22BT,APX1BT,APX2BT,
     &             THICKB,CAMBRB )
C
C==========================================
      ELSEIF(COMAND.EQ.'XYCM') THEN
       IF(NINPUT.GE.2) THEN
        XCMREF = RINPUT(1)
        YCMREF = RINPUT(2)
       ELSE
        CALL ASKR('Enter new CM reference X^',XCMREF)
        CALL ASKR('Enter new CM reference Y^',YCMREF)
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'BEND') THEN
        IF(N.EQ.0) THEN
         WRITE(*,*)
         WRITE(*,*) '***  No airfoil available  ***'
         GO TO 500
        ENDIF
C
        CALL BENDUMP(N,X,Y)
C
C===============================================
      ELSEIF(COMAND.EQ.'BENP') THEN
        IF(N.EQ.0) THEN
         WRITE(*,*)
         WRITE(*,*) '***  No airfoil available  ***'
         GO TO 500
        ENDIF
C
        DO I = 1, N
          W1(I) = 1.0
        ENDDO
        CALL BENDUMP2(N,X,Y,W1)
C
C===============================================
      ELSEIF(COMAND.EQ.'PCOP') THEN
       CALL ABCOPY(.TRUE.)
ccc       CALL PANPLT
C
C===============================================
      ELSEIF(COMAND.EQ.'PANE') THEN
       CALL PANGEN(.TRUE.)
ccc       CALL PANPLT
C
C===============================================
      ELSEIF(COMAND.EQ.'PPAR') THEN
       CALL GETPAN
C
C===============================================
      ELSEIF(COMAND.EQ.'PLOP') THEN
       CALL OPLSET(IDEV,IDEVRP,IPSLU,
     &             SIZE,PLOTAR,
     &             XMARG,YMARG,XPAGE,YPAGE,
     &             CH,SCRNFR,LCURS,LLAND,ICOLS)
C
C===============================================
      ELSEIF(COMAND.EQ.'WDEF') THEN
       LU = 8
       IF(COMARG(1:1).EQ.' ') THEN
        FNAME = 'xfoil.def'
       ELSE
        FNAME = COMARG
       ENDIF
       CALL STRIP(FNAME,NFN)
       OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=703)
       WRITE(*,701) FNAME(1:NFN)
 701   FORMAT(/'  File  ', A, '  exists.  Overwrite?  Y')
       READ(*,1000) ANS
       IF(INDEX('Nn',ANS).EQ.0) GO TO 706
       WRITE(*,*)
       WRITE(*,*) 'No action taken'
       CLOSE(LU)
C
 703   OPEN(LU,FILE=FNAME,STATUS='UNKNOWN')
 706   CALL WRTDEF(LU)
       WRITE(*,708) FNAME(1:NFN)
 708   FORMAT(/'  File  ', A, '  written')
       CLOSE(LU)
C
C===============================================
      ELSEIF(COMAND.EQ.'RDEF') THEN
       IF(COMARG(1:1).EQ.' ') THEN
        FNAME = 'xfoil.def'
       ELSE
        FNAME = COMARG
       ENDIF
C
       LU = 8
       CALL GETDEF(LU,FNAME, .FALSE.)
C
C===============================================
      ELSEIF(COMAND.EQ.'NAME') THEN
       IF(COMARG.EQ.' ') THEN
        CALL NAMMOD(NAME,0,-1)
       ELSE
        NAME = COMARG
       ENDIF
       CALL STRIP(NAME,NNAME)
C
C===============================================
      ELSEIF(COMAND.EQ.'NINC') THEN
       CALL NAMMOD(NAME,1,1)
       CALL STRIP(NAME,NNAME)
C
C===============================================
      ELSEIF(COMAND.EQ.'Z   ') THEN
       IF(LPLOT) THEN
        CALL USETZOOM(.TRUE.,.TRUE.)
        CALL REPLOT(IDEV)
       ENDIF
C
C===============================================
      ELSEIF(COMAND.EQ.'U   ') THEN
       IF(LPLOT) THEN
        CALL CLRZOOM
        CALL REPLOT(IDEV)
       ENDIF
C
C===============================================
      ELSE
       WRITE(*,1050) COMAND
 1050  FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
C
      ENDIF
C
C===============================================
      GO TO 500
C
 1000 FORMAT(A)
      END ! XFOIL

 
      SUBROUTINE INIT
C---------------------------------------------------
C     Variable initialization/default routine.
C     See file XFOIL.INC for variable description.
C---------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      PI = 4.0*ATAN(1.0)
      HOPI = 0.50/PI
      QOPI = 0.25/PI
      DTOR = PI/180.0
C
C---- default Cp/Cv (air)
      GAMMA = 1.4
      GAMM1 = GAMMA - 1.0
C
C---- set unity freestream speed
      QINF = 1.0
C
C---- initialize freestream Mach number to zero
      MATYP = 1
      MINF1 = 0.
C
      ALFA = 0.0
      COSA = 1.0
      SINA = 0.0
C
      DO 10 I=1, IQX
        GAMU(I,1) = 0.
        GAMU(I,2) = 0.
        GAM(I) = 0.
        GAM_A(I) = 0.
   10 CONTINUE
      PSIO = 0.
C
      CL = 0.
      CM = 0.
      CD = 0.
C
      SIGTE = 0.0
      GAMTE = 0.0
      SIGTE_A = 0.
      GAMTE_A = 0.
C
      DO 20 I=1, IZX
        SIG(I) = 0.
   20 CONTINUE
C
      NQSP = 0
      DO 30 K=1, IPX
        ALQSP(K) = 0.
        CLQSP(K) = 0.
        CMQSP(K) = 0.
        DO 302 I=1, IBX
          QSPEC(I,K) = 0.
 302    CONTINUE
 30   CONTINUE
C
      AWAKE = 0.0
      AVISC = 0.0
C
      KIMAGE = 1
      YIMAGE = -10.0
      LIMAGE = .FALSE.
C
C---- output coordinate file delimiters
C-    KDELIM =  0  blanks
C-           =  1  commas
C-           =  2  tabs
      KDELIM = 0
C
      LGAMU  = .FALSE.
      LQINU  = .FALSE.
      LVISC  = .FALSE.
      LWAKE  = .FALSE.
      LPACC  = .FALSE.
      LBLINI = .FALSE.
      LIPAN  = .FALSE.
      LQAIJ  = .FALSE.
      LADIJ  = .FALSE.
      LWDIJ  = .FALSE.
      LCPXX  = .FALSE.
      LQVDES = .FALSE.
      LQSPEC = .FALSE.
      LQREFL = .FALSE.
      LVCONV = .FALSE.
      LCPREF = .FALSE.
      LFOREF = .FALSE.
      LPFILE = .FALSE.
      LPFILX = .FALSE.
      LPPSHO = .FALSE.
      LBFLAP = .FALSE.
      LFLAP  = .FALSE.
      LEIW   = .FALSE.
      LSCINI = .FALSE.
      LPLOT  = .FALSE.
      LCLIP  = .FALSE.
      LVLAB  = .TRUE.
      LCMINP = .FALSE.
      LHMOMP = .FALSE.
      LFREQP = .TRUE.
C
      LCURS  = .TRUE.
      LLAND  = .TRUE.
      LGSAME = .FALSE.
C
      LGPARM = .TRUE.
      LPLCAM = .FALSE.
C
C---- input airfoil will not be normalized
      LNORM = .FALSE.
C
C---- airfoil will not be forced symmetric
      LQSYM = .FALSE.
      LGSYM = .FALSE.
C
C---- endpoint slopes will be matched
      LQSLOP = .TRUE.
      LGSLOP = .TRUE.
      LCSLOP = .TRUE.
C
C---- grids on Qspec(s) and buffer airfoil geometry plots will be plotted
      LQGRID = .TRUE.
      LGGRID = .TRUE.
      LGTICK = .TRUE.
C
C---- no grid on Cp plots
      LCPGRD = .FALSE.
C
C---- overlay inviscid Cp on viscous Cp(x) plot
      LCPINV = .TRUE.

C---- grid and no symbols are to be used on BL variable plots
      LBLGRD = .TRUE.
      LBLSYM = .FALSE.
C
C---- buffer and current airfoil flap hinge coordinates
      XBF = 0.0
      YBF = 0.0
      XOF = 0.0
      YOF = 0.0
C
      NCPREF = 0
C                                               n
C---- circle plane array size (257, or largest 2  + 1 that will fit array size)
      ANN = LOG(FLOAT((2*IQX)-1))/LOG(2.0)
      NN = INT( ANN + 0.00001 )
      NC1 = 2**NN + 1
      NC1 = MIN( NC1 , 257 )
C
C---- default paneling parameters
      NPAN = 160
      CVPAR = 1.0
      CTERAT = 0.15
      CTRRAT = 0.2
C
C---- default paneling refinement zone x/c endpoints
      XSREF1 = 1.0
      XSREF2 = 1.0
      XPREF1 = 1.0
      XPREF2 = 1.0
C
C---- no polars present to begin with
      NPOL = 0
      IPACT = 0
      DO IP = 1, NPX
        PFNAME(IP) = ' '
        PFNAMX(IP) = ' '
      ENDDO
C
C---- no reference polars
      NPOLREF = 0
C
C---- plot aspect ratio, character size
      PLOTAR = 0.55
      CH     = 0.015
C
C---- airfoil node tick-mark size (as fraction of arc length)
      GTICK = 0.0005
C
C---- Cp limits in  Cp vs x  plot
      CPMAX =  1.0
      CPMIN = -2.0
      CPDEL = -0.5
      PFAC = PLOTAR/(CPMAX-CPMIN)
C
C---- Ue limits in  Ue vs x  plot
      UEMAX =  1.8
      UEMIN = -1.0
      UEDEL =  0.2
      UFAC = PLOTAR/(UEMAX-UEMIN)
C
C---- DCp limits in CAMB loading plot
      YPMIN = -0.4
      YPMAX =  0.4
C
C---- scaling factor for Cp vector plot
      VFAC = 0.25
C
C---- offsets and scale factor for airfoil in  Cp vs x  plot
      XOFAIR = 0.09
      YOFAIR = -.01
      FACAIR = 0.70
C
C---- u/Qinf scale factor for profile plotting
      UPRWT = 0.02
C
C---- polar plot options, grid, list, legend, no CDW
      LPGRID = .TRUE.
      LPCDW  = .FALSE.
      LPLIST = .TRUE.
      LPLEGN = .TRUE.
      LAECEN = .FALSE.
      LPCDH   = .FALSE.
      LPCMDOT = .FALSE.
C
C---- axis limits and annotation deltas for polar plot
      CPOLPLF(1,ICD) = 0.0
      CPOLPLF(2,ICD) = 0.04
      CPOLPLF(3,ICD) = 0.01
C        
      CPOLPLF(1,ICL) = 0.
      CPOLPLF(2,ICL) = 1.5
      CPOLPLF(3,ICL) = 0.5
C        
      CPOLPLF(1,ICM) = -0.25
      CPOLPLF(2,ICM) =  0.0
      CPOLPLF(3,ICM) =  0.05
C        
      CPOLPLF(1,IAL) = -4.0
      CPOLPLF(2,IAL) = 10.0
      CPOLPLF(3,IAL) =  2.0
C
C---- widths of plot boxes in polar plot page
      XCDWID = 0.45
      XALWID = 0.25
      XOCWID = 0.20
C
C---- line style and color index for each polar
C
C     1  *****************************  SOLID
C     2  **** **** **** **** **** ****  LONG DASHED
C     3  ** ** ** ** ** ** ** ** ** **  SHORT DASHED
C     4  * * * * * * * * * * * * * * *  DOTTED
C     5  ***** * ***** * ***** * *****  DASH-DOT
C     6  ***** * * ***** * * ***** * *  DASH-DOT-DOT
C     7  ***** * * * ***** * * * *****  DASH-DOT-DOT-DOT
C     8  **** **** * * **** **** * *    DASH-DASH-DOT-DOT
C
C     3  red
C     4  orange
C     5  yellow
C     6  green
C     7  cyan
C     8  blue
C     9  violet
C    10  magenta
C
      DO IP=1, NPX
cc        ILINP(IP) = 1 + MOD(IP-1,8)
cc        ICOLP(IP) = 3 + MOD(IP-1,8)
C
C------ normally solid, going to dashed after IP=7
        ILINP(IP) = 1 + (IP-1)/7
C
C------ skip yellow (hard to see on white background)
        ICOLP(IP) = 3 + MOD(IP-1,7)
        IF(ICOLP(IP) .GE. 5) ICOLP(IP) = ICOLP(IP) + 1
      ENDDO
C
C---- polar variables to be written to polar save file
      IPOL(1) = IAL
      IPOL(2) = ICL
      IPOL(3) = ICD
      IPOL(4) = ICP
      IPOL(5) = ICM
      NIPOL = 5
      NIPOL0 = 5
C
      JPOL(1) = JTN
      JPOL(2) = JTI
      NJPOL = 2
C
C---- default Cm reference location
      XCMREF = 0.25
      YCMREF = 0.
C
C---- default viscous parameters
      RETYP = 1
      REINF1 = 0.
      ACRIT(1) = 9.0
      ACRIT(2) = 9.0
      XSTRIP(1) = 1.0
      XSTRIP(2) = 1.0
      XOCTR(1) = 1.0
      XOCTR(2) = 1.0
      YOCTR(1) = 0.
      YOCTR(2) = 0.
      WAKLEN = 1.0
C
      IDAMP = 0
C
C---- select default BL plotting coordinate (can be changed in VPLO)
      IXBLP = 1   !  x
ccc   IXBLP = 2   !  s
C
C---- set BL calibration parameters
      CALL BLPINI
C
C---- Newton iteration limit
      ITMAX = 20
C
C---- max number of unconverged sequence points for early exit
      NSEQEX = 4
C
C---- drop tolerance for BL system solver
      VACCEL = 0.01
C
C---- inverse-mapping auto-filter level
      FFILT = 0.0
C
C---- default overlay airfoil filename
      ONAME = ' '
C
C---- default filename prefix
      PREFIX = ' '
C
C---- Plotting flag
      IDEV = 1   ! X11 window only
c     IDEV = 2                  ! B&W PostScript output file only (no color)
c     IDEV = 3   ! both X11 and B&W PostScript file
c     IDEV = 4   ! Color PostScript output file only 
c     IDEV = 5   ! both X11 and Color PostScript file 
C
C---- Re-plotting flag (for hardcopy)
c     IDEVRP = 2   ! B&W PostScript
      IDEVRP = 4   ! Color PostScript
C
C---- PostScript output logical unit and file specification
      IPSLU = 0  ! output to file  plot.ps   on LU 4    (default case)
c     IPSLU = ?  ! output to file  plot?.ps  on LU 10+?
C
C---- screen fraction taken up by plot window upon opening
      SCRNFR = 0.80
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
C---- set top and bottom-side colors
cc      ICOLS(1) = 5
cc      ICOLS(2) = 7
      ICOLS(1) = 8
      ICOLS(2) = 3
C
C   3  red
C   4  orange
C   5  yellow
C   6  green
C   7  cyan
C   8  blue
C   9  violet
C  10  magenta
C
C
      CALL PLINITIALIZE
C
C---- set up color spectrum
      NCOLOR = 64
      CALL COLORSPECTRUMHUES(NCOLOR,'RYGCBM')
C
C
      NNAME  = 32
      NAME   = '                                '
CCC             12345678901234567890123456789012
C
C---- MSES domain parameters (not used in XFOIL)
      ISPARS = ' -2.0  3.0  -2.5  3.5'
C
C---- set MINF, REINF, based on current CL-dependence
      CALL MRCL(1.0,MINF_CL,REINF_CL)
C
C---- set various compressibility parameters from MINF
      CALL COMSET
C
      RETURN
      END ! INIT


      SUBROUTINE MRCL(CLS,M_CLS,R_CLS)
C-------------------------------------------
C     Sets actual Mach, Reynolds numbers
C     from unit-CL values and specified CLS
C     depending on MATYP,RETYP flags.
C-------------------------------------------
      INCLUDE 'XFOIL.INC'
      REAL M_CLS
C
      CLA = MAX( CLS , 0.000001 )
C
      IF(RETYP.LT.1 .OR. RETYP.GT.3) THEN
        WRITE(*,*) 'MRCL:  Illegal Re(CL) dependence trigger.'
        WRITE(*,*) '       Setting fixed Re.'
        RETYP = 1
      ENDIF
      IF(MATYP.LT.1 .OR. MATYP.GT.3) THEN
        WRITE(*,*) 'MRCL:  Illegal Mach(CL) dependence trigger.'
        WRITE(*,*) '       Setting fixed Mach.'
        MATYP = 1
      ENDIF
C
C
      IF(MATYP.EQ.1) THEN
C
        MINF  = MINF1
        M_CLS = 0.
C
      ELSE IF(MATYP.EQ.2) THEN
C
        MINF  =  MINF1/SQRT(CLA)
        M_CLS = -0.5*MINF/CLA
C
      ELSE IF(MATYP.EQ.3) THEN
C
        MINF  = MINF1
        M_CLS = 0.
C
      ENDIF
C
C
      IF(RETYP.EQ.1) THEN
C
        REINF = REINF1
        R_CLS = 0.
C
      ELSE IF(RETYP.EQ.2) THEN
C
        REINF =  REINF1/SQRT(CLA)
        R_CLS = -0.5*REINF/CLA
C
      ELSE IF(RETYP.EQ.3) THEN
C
        REINF =  REINF1/CLA
        R_CLS = -REINF /CLA
C
      ENDIF
C
C
      IF(MINF .GE. 0.99) THEN
        WRITE(*,*)
        WRITE(*,*) 'MRCL: CL too low for chosen Mach(CL) dependence'
        WRITE(*,*) '      Aritificially limiting Mach to  0.99'
        MINF = 0.99
        M_CLS = 0.
      ENDIF
C
      RRAT = 1.0
      IF(REINF1 .GT. 0.0) RRAT = REINF/REINF1
C
      IF(RRAT .GT. 100.0) THEN
        WRITE(*,*)
        WRITE(*,*) 'MRCL: CL too low for chosen Re(CL) dependence'
        WRITE(*,*) '      Aritificially limiting Re to ',REINF1*100.0
        REINF = REINF1*100.0
        R_CLS = 0.
      ENDIF
C
      RETURN
      END ! MRCL


 
      SUBROUTINE GETDEF(LU,FILNAM,LASK)
      CHARACTER*(*) FILNAM
      LOGICAL LASK
C-----------------------------------------------------
C     Reads in default parameters from file xfoil.def
C     If LASK=t, ask user if file is to be read.
C-----------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LCOLOR
      CHARACTER*1 ANS
C
 1000 FORMAT(A)
C
      OPEN(LU,FILE=FILNAM,STATUS='OLD',ERR=90)
      IF(LASK) THEN
       WRITE(*,1050) FILNAM
 1050  FORMAT(/'  Read settings from file  ', A, ' ?  Y')
       READ(*,1000) ANS
       IF(INDEX('Nn',ANS).NE.0) THEN
        CLOSE(LU)
        RETURN
       ENDIF
      ENDIF
C
      CLMIN = CPOLPLF(1,ICL)
      CLMAX = CPOLPLF(2,ICL)
      CLDEL = CPOLPLF(3,ICL)
C                 
      CDMIN = CPOLPLF(1,ICD)
      CDMAX = CPOLPLF(2,ICD)
      CDDEL = CPOLPLF(3,ICD)
C                 
      ALMIN = CPOLPLF(1,IAL)
      ALMAX = CPOLPLF(2,IAL)
      ALDEL = CPOLPLF(3,IAL)
C
      CMMIN = CPOLPLF(1,ICM)
      CMMAX = CPOLPLF(2,ICM)
      CMDEL = CPOLPLF(3,ICM)
C                 
C---- default paneling parameters (viscous)
      READ(LU,*,ERR=80) NPAN, CVPAR, CTERAT, CTRRAT
      READ(LU,*,ERR=80) XSREF1, XSREF2, XPREF1, XPREF2
C
C---- plotting parameters
      READ(LU,*,ERR=80) SIZE, PLOTAR, CH, SCRNFR
C
C---- plot sizes
      READ(LU,*,ERR=80) XPAGE, YPAGE, XMARG, YMARG
C
C---- plot flags
      READ(LU,*,ERR=80) LCOLOR, LCURS
C
C---- Cp limits in  Cp vs x  plot
      READ(LU,*,ERR=80) CPMAX, CPMIN, CPDEL
      PFAC = PLOTAR/(CPMAX-CPMIN)
C
C---- airfoil x-offset and scale factor in Cp vs x plot, BL profile weight
      READ(LU,*,ERR=80) XOFAIR, FACAIR, UPRWT
C
C---- polar plot CL,CD,alpha,CM  min,max,delta 
      READ(LU,*,ERR=80) (CPOLPLF(K,ICL), K=1, 3)
      READ(LU,*,ERR=80) (CPOLPLF(K,ICD), K=1, 3)
      READ(LU,*,ERR=80) (CPOLPLF(K,IAL), K=1, 3)
      READ(LU,*,ERR=80) (CPOLPLF(K,ICM), K=1, 3)
C
C---- default Mach and viscous parameters
      READ(LU,*,ERR=80) MATYP, MINF1, VACCEL
      READ(LU,*,ERR=80) RETYP, RMILL, ACRIT(1), ACRIT(2)
      READ(LU,*,ERR=80) XSTRIP(1), XSTRIP(2)
C
      IF(     LCOLOR) IDEVRP = 4
      IF(.NOT.LCOLOR) IDEVRP = 2
C
      REINF1 = RMILL * 1.0E6
C
C---- set MINF, REINF
      CALL MRCL(1.0,MINF_CL,REINF_CL)
C
C---- set various compressibility parameters from new MINF
      CALL COMSET
C
      CLOSE(LU)
      WRITE(*,1600) FILNAM
 1600 FORMAT(/' Default parameters read in from file  ', A,':' /)
      CALL WRTDEF(6)
      RETURN
C
 80   CONTINUE
      CLOSE(LU)
      WRITE(*,1800) FILNAM
 1800 FORMAT(/' File  ', A,'  read error'
     &       /' Settings may have been changed')
      RETURN
C
 90   CONTINUE
      WRITE(*,1900) FILNAM
 1900 FORMAT(/' File  ', A,'  not found')
      RETURN
C
      END ! GETDEF


 
      SUBROUTINE WRTDEF(LU)
C------------------------------------------
C     Writes default parameters to unit LU
C------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LCOLOR
C
      LCOLOR = IDEVRP.EQ.4
C
C---- default paneling parameters (viscous)
      WRITE(LU,1010) NPAN  , CVPAR , CTERAT, CTRRAT
      WRITE(LU,1020) XSREF1, XSREF2, XPREF1, XPREF2
C
C---- plotting parameters
      WRITE(LU,1030) SIZE, PLOTAR, CH, SCRNFR
C
C---- plot sizes
      WRITE(LU,1032) XPAGE, YPAGE, XMARG, YMARG
C
C---- plot flags
      WRITE(LU,1034) LCOLOR, LCURS
C
C---- Cp limits in  Cp vs x  plot
      WRITE(LU,1040) CPMAX, CPMIN, CPDEL
C
C---- x-offset and scale factor for airfoil on Cp vs x plot
      WRITE(LU,1050) XOFAIR, FACAIR, UPRWT
C
C---- polar plot CL,CD,alpha,CM  min,max,delta 
      WRITE(LU,1061) (CPOLPLF(K,ICL), K=1, 3)
      WRITE(LU,1062) (CPOLPLF(K,ICD), K=1, 3)
      WRITE(LU,1063) (CPOLPLF(K,IAL), K=1, 3)
      WRITE(LU,1064) (CPOLPLF(K,ICM), K=1, 3)
C
C---- default viscous parameters
      WRITE(LU,1071) MATYP  , MINF1        , VACCEL
      WRITE(LU,1072) RETYP  , REINF1/1.0E6 , ACRIT(1), ACRIT(2)
      WRITE(LU,1080) XSTRIP(1), XSTRIP(2)
C
      RETURN
C...............................................
 1010 FORMAT(1X,I5,4X,F9.4,F9.4,F9.4,' | Npan    PPanel  TErat  REFrat')
 1020 FORMAT(1X,F9.4 ,F9.4,F9.4,F9.4,' | XrefS1  XrefS2  XrefP1 XrefP2')
 1030 FORMAT(1X,F9.4 ,F9.4,F9.4,F9.4,' | Size    plotAR  CHsize ScrnFr')
 1032 FORMAT(1X,F9.4 ,F9.4,F9.4,F9.4,' | Xpage   Ypage   Xmargn Ymargn')
 1034 FORMAT(1X,L2,7X,L2,7X,9X , 9X ,' | Lcolor  Lcursor'              )
 1040 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | CPmax   CPmin   CPdel'        )
 1050 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | XoffAir ScalAir BLUwt'        )
 1061 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | CLmin   CLmax   CLdel'        )
 1062 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | CDmin   CDmax   CDdel'        )
 1063 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | ALmin   ALmax   ALdel'        )
 1064 FORMAT(1X,F9.4 ,F9.4,F9.4, 9X ,' | CMmin   CMmax   CMdel'        )
 1071 FORMAT(1X,I3,6X,F9.4,F9.4, 9X ,' | MAtype  Mach    Vaccel'       )
 1072 FORMAT(1X,I3,6X,F9.4,F9.4,F9.4,' | REtype  Re/10^6 Ncrit1 Ncrit2')
 1080 FORMAT(1X,F9.4 ,F9.4, 9X , 9X ,' | XtripT  XtripB'               )
      END ! WRTDEF


      SUBROUTINE COMSET
      INCLUDE 'XFOIL.INC'
C
C---- set Karman-Tsien parameter TKLAM
      BETA = SQRT(1.0 - MINF**2)
      BETA_MSQ = -0.5/BETA
C
      TKLAM   = MINF**2 / (1.0 + BETA)**2
      TKL_MSQ =     1.0 / (1.0 + BETA)**2
     &    - 2.0*TKLAM/ (1.0 + BETA) * BETA_MSQ
C
C---- set sonic Pressure coefficient and speed
      IF(MINF.EQ.0.0) THEN
       CPSTAR = -999.0
       QSTAR = 999.0
      ELSE
       CPSTAR = 2.0 / (GAMMA*MINF**2)
     &        * (( (1.0 + 0.5*GAMM1*MINF**2)
     &            /(1.0 + 0.5*GAMM1        ))**(GAMMA/GAMM1) - 1.0)
       QSTAR = QINF/MINF
     &       * SQRT( (1.0 + 0.5*GAMM1*MINF**2)
     &              /(1.0 + 0.5*GAMM1        ) )
      ENDIF
C
      RETURN
      END ! COMSET


      SUBROUTINE CPCALC(N,Q,QINF,MINF,CP)
C---------------------------------------------
C     Sets compressible Cp from speed.
C---------------------------------------------
      DIMENSION Q(N),CP(N)
      REAL MINF
C
      LOGICAL DENNEG
C
      BETA = SQRT(1.0 - MINF**2)
      BFAC = 0.5*MINF**2 / (1.0 + BETA)
C
      DENNEG = .FALSE.
C
      DO 20 I=1, N
        CPINC = 1.0 - (Q(I)/QINF)**2
        DEN = BETA + BFAC*CPINC
        CP(I) = CPINC / DEN
        IF(DEN .LE. 0.0) DENNEG = .TRUE.
  20  CONTINUE
C
      IF(DENNEG) THEN
       WRITE(*,*)
       WRITE(*,*) 'CPCALC: Local speed too large. ',
     &            'Compressibility corrections invalid.'
      ENDIF
C
      RETURN
      END ! CPCALC

 
      SUBROUTINE CLCALC(N,X,Y,GAM,GAM_A,ALFA,MINF,QINF, 
     &                  XREF,YREF,
     &                  CL,CM,CDP, CL_ALF,CL_MSQ)
C-----------------------------------------------------------
C     Integrates surface pressures to get CL and CM.
C     Integrates skin friction to get CDF.
C     Calculates dCL/dAlpha for prescribed-CL routines.
C-----------------------------------------------------------
      DIMENSION X(N),Y(N), GAM(N), GAM_A(N)
      REAL MINF
C
ccC---- moment-reference coordinates
cc      XREF = 0.25
cc      YREF = 0.
C
      SA = SIN(ALFA)
      CA = COS(ALFA)
C
      BETA = SQRT(1.0 - MINF**2)
      BETA_MSQ = -0.5/BETA
C
      BFAC     = 0.5*MINF**2 / (1.0 + BETA)
      BFAC_MSQ = 0.5         / (1.0 + BETA)
     &         - BFAC        / (1.0 + BETA) * BETA_MSQ
C
      CL = 0.0
      CM = 0.0

      CDP = 0.0
C
      CL_ALF = 0.
      CL_MSQ = 0.
C
      I = 1
      CGINC = 1.0 - (GAM(I)/QINF)**2
      CPG1     = CGINC/(BETA + BFAC*CGINC)
      CPG1_MSQ = -CPG1/(BETA + BFAC*CGINC)*(BETA_MSQ + BFAC_MSQ*CGINC)
C
      CPI_GAM = -2.0*GAM(I)/QINF**2
      CPC_CPI = (1.0 - BFAC*CPG1)/ (BETA + BFAC*CGINC)
      CPG1_ALF = CPC_CPI*CPI_GAM*GAM_A(I)
C
      DO 10 I=1, N
        IP = I+1
        IF(I.EQ.N) IP = 1
C
        CGINC = 1.0 - (GAM(IP)/QINF)**2
        CPG2     = CGINC/(BETA + BFAC*CGINC)
        CPG2_MSQ = -CPG2/(BETA + BFAC*CGINC)*(BETA_MSQ + BFAC_MSQ*CGINC)
C
        CPI_GAM = -2.0*GAM(IP)/QINF**2
        CPC_CPI = (1.0 - BFAC*CPG2)/ (BETA + BFAC*CGINC)
        CPG2_ALF = CPC_CPI*CPI_GAM*GAM_A(IP)
C
        DX = (X(IP) - X(I))*CA + (Y(IP) - Y(I))*SA
        DY = (Y(IP) - Y(I))*CA - (X(IP) - X(I))*SA
        DG = CPG2 - CPG1
C
        AX = (0.5*(X(IP)+X(I))-XREF)*CA + (0.5*(Y(IP)+Y(I))-YREF)*SA
        AY = (0.5*(Y(IP)+Y(I))-YREF)*CA - (0.5*(X(IP)+X(I))-XREF)*SA
        AG = 0.5*(CPG2 + CPG1)
C
        DX_ALF = -(X(IP) - X(I))*SA + (Y(IP) - Y(I))*CA
        AG_ALF = 0.5*(CPG2_ALF + CPG1_ALF)
        AG_MSQ = 0.5*(CPG2_MSQ + CPG1_MSQ)
C
        CL     = CL     + DX* AG
        CDP    = CDP    - DY* AG
        CM     = CM     - DX*(AG*AX + DG*DX/12.0)
     &                  - DY*(AG*AY + DG*DY/12.0)
C
        CL_ALF = CL_ALF + DX*AG_ALF + AG*DX_ALF
        CL_MSQ = CL_MSQ + DX*AG_MSQ
C
        CPG1 = CPG2
        CPG1_ALF = CPG2_ALF
        CPG1_MSQ = CPG2_MSQ
   10 CONTINUE
C
      RETURN
      END ! CLCALC



      SUBROUTINE CDCALC
      INCLUDE 'XFOIL.INC'
C
      SA = SIN(ALFA)
      CA = COS(ALFA)
C
      IF(LVISC .AND. LBLINI) THEN
C
C----- set variables at the end of the wake
       THWAKE = THET(NBL(2),2)
       URAT   = UEDG(NBL(2),2)/QINF
       UEWAKE = UEDG(NBL(2),2) * (1.0-TKLAM) / (1.0 - TKLAM*URAT**2)
       SHWAKE = DSTR(NBL(2),2)/THET(NBL(2),2)
C
C----- extrapolate wake to downstream infinity using Squire-Young relation
C      (reduces errors of the wake not being long enough)
       CD = 2.0*THWAKE * (UEWAKE/QINF)**(0.5*(5.0+SHWAKE))
C
      ELSE
C
       CD = 0.0
C
      ENDIF
C
C---- calculate friction drag coefficient
      CDF = 0.0
      DO 20 IS=1, 2
        DO 205 IBL=3, IBLTE(IS)
          I  = IPAN(IBL  ,IS)
          IM = IPAN(IBL-1,IS)
          DX = (X(I) - X(IM))*CA + (Y(I) - Y(IM))*SA
          CDF = CDF + 0.5*(TAU(IBL,IS)+TAU(IBL-1,IS))*DX * 2.0/QINF**2
 205    CONTINUE
 20   CONTINUE
C
      RETURN
      END ! CDCALC



      SUBROUTINE LOAD(FILNAM,ITYPE)
C------------------------------------------------------
C     Reads airfoil file into buffer airfoil
C     and does various initial processesing on it.
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) FILNAM
C
      FNAME = FILNAM
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter filename^',FNAME)
C
      LU = 9
      CALL AREAD(LU,FNAME,IBX,XB,YB,NB,NAME,ISPARS,ITYPE,1)
      IF(ITYPE.EQ.0) RETURN
C
      IF(ITYPE.EQ.1) CALL ASKS('Enter airfoil name^',NAME)
      CALL STRIP(NAME,NNAME)
C
C---- set default prefix for other filenames
      KDOT = INDEX(FNAME,'.')
      IF(KDOT.EQ.0) THEN
       PREFIX = FNAME
      ELSE
       PREFIX = FNAME(1:KDOT-1)
      ENDIF
      CALL STRIP(PREFIX,NPREFIX)
C
C---- calculate airfoil area assuming counterclockwise ordering
      AREA = 0.0
      DO 50 I=1, NB
        IP = I+1
        IF(I.EQ.NB) IP = 1
        AREA = AREA + 0.5*(YB(I)+YB(IP))*(XB(I)-XB(IP))
   50 CONTINUE
C
      IF(AREA.GE.0.0) THEN
       LCLOCK = .FALSE.
       WRITE(*,1010) NB
      ELSE
C----- if area is negative (clockwise order), reverse coordinate order
       LCLOCK = .TRUE.
       WRITE(*,1011) NB
       DO 55 I=1, NB/2
         XTMP = XB(NB-I+1)
         YTMP = YB(NB-I+1)
         XB(NB-I+1) = XB(I)
         YB(NB-I+1) = YB(I)
         XB(I) = XTMP
         YB(I) = YTMP
   55  CONTINUE
      ENDIF
C
      IF(LNORM) THEN
       CALL NORM(XB,XBP,YB,YBP,SB,NB)
       WRITE(*,1020)
      ENDIF
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB, W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      XBLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YBLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XBTE = 0.5*(XB(1) + XB(NB))
      YBTE = 0.5*(YB(1) + YB(NB))
C
      WRITE(*,1050) XBLE,YBLE, CHORDB,
     &              XBTE,YBTE
C
C---- set reasonable MSES domain parameters for non-MSES coordinate file
      IF(ITYPE.LE.2 .AND. ISPARS.EQ.' ') THEN
        XBLE = SEVAL(SBLE,XB,XBP,SB,NB)
        YBLE = SEVAL(SBLE,YB,YBP,SB,NB)
        XINL = XBLE - 2.0*CHORDB
        XOUT = XBLE + 3.0*CHORDB
        YBOT = YBLE - 2.5*CHORDB
        YTOP = YBLE + 3.5*CHORDB
        XINL = AINT(20.0*ABS(XINL/CHORDB)+0.5)/20.0 * SIGN(CHORDB,XINL)
        XOUT = AINT(20.0*ABS(XOUT/CHORDB)+0.5)/20.0 * SIGN(CHORDB,XOUT)
        YBOT = AINT(20.0*ABS(YBOT/CHORDB)+0.5)/20.0 * SIGN(CHORDB,YBOT)
        YTOP = AINT(20.0*ABS(YTOP/CHORDB)+0.5)/20.0 * SIGN(CHORDB,YTOP)
        WRITE(ISPARS,1005) XINL, XOUT, YBOT, YTOP
 1005   FORMAT(1X, 4F8.2 )
      ENDIF
C
C---- wipe out old flap hinge location
      XBF = 0.0
      YBF = 0.0
      LBFLAP = .FALSE.
C
C---- wipe out off-design alphas, CLs
cc      NALOFF = 0
cc      NCLOFF = 0
C
      RETURN
C...............................................................
 1010 FORMAT(/' Number of input coordinate points:', I4
     &       /' Counterclockwise ordering')
 1011 FORMAT(/' Number of input coordinate points:', I4
     &       /' Clockwise ordering')
 1020 FORMAT(/' Airfoil has been normalized')
 1050 FORMAT(/'  LE  x,y  =', 2F10.5,'  |   Chord =',F10.5
     &       /'  TE  x,y  =', 2F10.5,'  |'                 )
      END ! LOAD


 
      SUBROUTINE SAVE(IFTYP,FNAME1)
C--------------------------------
C     Writes out current airfoil 
C--------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) FNAME1
C
      CHARACTER*1 ANS, DELIM
      CHARACTER*128 LINE
C
      IF    (KDELIM.EQ.0) THEN
       DELIM = ' '
      ELSEIF(KDELIM.EQ.1) THEN
       DELIM = ','
      ELSEIF(KDELIM.EQ.2) THEN
       DELIM = CHAR(9)
      ELSE
       WRITE(*,*) '? Illegal delimiter.  Using blank.'
       DELIM = ' '
      ENDIF
C
C
      LU = 2
C
C---- get output filename if it was not supplied
      IF(FNAME1(1:1) .NE. ' ') THEN
       FNAME = FNAME1
      ELSE
       CALL ASKS('Enter output filename^',FNAME)
      ENDIF
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=5)
      WRITE(*,*)
      WRITE(*,*) 'Output file exists.  Overwrite?  Y'
      READ(*,1000) ANS
      IF(INDEX('Nn',ANS).EQ.0) GO TO 6
C
      CLOSE(LU)
      WRITE(*,*) 'Current airfoil not saved.'
      RETURN
C
 5    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=90)
 6    REWIND(LU)
C
      IF(IFTYP.GE.1) THEN
C----- write name to first line
       WRITE(LU,1000) NAME(1:NNAME)
      ENDIF
C
      IF(IFTYP.GE.2) THEN
C----- write MSES domain parameters to second line
       DO K=80, 1, -1
         IF(INDEX(ISPARS(K:K),' ') .NE. 1) GO TO 11
       ENDDO
 11    CONTINUE
C
       WRITE(LU,1000) ISPARS(1:K)
      ENDIF
C
      IF(LCLOCK) THEN
C----- write out in clockwise order (reversed from internal XFOIL order)
       IBEG = N
       IEND = 1
       INCR = -1
      ELSE
C----- write out in counterclockwise order (same as internal XFOIL order)
       IBEG = 1
       IEND = N
       INCR = 1
      ENDIF
C
      IF(IFTYP.EQ.-1) THEN
       DO I = IBEG, IEND, INCR
         WRITE(LU,1400) INT(X(I)+SIGN(0.5,X(I))), 
     &                  INT(Y(I)+SIGN(0.5,Y(I)))
       ENDDO
C
      ELSE
       DO I = IBEG, IEND, INCR
         IF(KDELIM .EQ. 0) THEN
          WRITE(LU,1100) X(I), Y(I)
C
         ELSE
          WRITE(LINE,1200) X(I), DELIM, Y(I)
          CALL BSTRIP(LINE,NLINE)
          WRITE(LU,1000) LINE(1:NLINE)
         ENDIF
       ENDDO
      ENDIF
C
      CLOSE(LU)
      RETURN
C
 90   WRITE(*,*) 'Bad filename.'
      WRITE(*,*) 'Current airfoil not saved.'
      RETURN
C
 1000 FORMAT(A)
 1100 FORMAT(1X,G15.7, G15.7)
 1200 FORMAT(1X,F10.6, A, F10.6)
 1400 FORMAT(1X,  I12, I12)
      END ! SAVE



      SUBROUTINE MSAVE(FNAME1)
C------------------------------------------
C     Writes out current airfoil as one 
C     element in a multielement MSES file.
C------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) FNAME1
C
      CHARACTER*80 NAME1, ISPARS1
C
      PARAMETER (NEX=5)
      DIMENSION NTMP(NEX)
      DIMENSION XTMP(2*IQX,NEX), YTMP(2*IQX,NEX)
      EQUIVALENCE (Q(1,1),XTMP(1,1)), (Q(1,IQX/2),YTMP(1,1))
C
      LU = 2
C
C---- get output filename if it was not supplied
      IF(FNAME1(1:1) .NE. ' ') THEN
       FNAME = FNAME1
      ELSE
       CALL ASKS('Enter output filename for element replacement^',FNAME)
      ENDIF
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=9005)
C
      READ(LU,1000,ERR=9010) NAME1
      READ(LU,1000,ERR=9010) ISPARS1
C
      DO NN1=80, 2, -1
        IF(NAME1(NN1:NN1) .NE. ' ') GO TO 10
      ENDDO
 10   CONTINUE
C
      DO NI1=80, 2, -1
        IF(ISPARS1(NI1:NI1) .NE. ' ') GO TO 20
      ENDDO
 20   CONTINUE
C
C---- read in existing airfoil coordinates
   40 DO 55 IEL=1, NEX
        DO 50 I=1, 2*IQX+1
          READ(LU,*,END=56) XTMP(I,IEL), YTMP(I,IEL)
          IF(XTMP(I,IEL).EQ.999.0) THEN
           NTMP(IEL) = I-1
           GO TO 55
          ENDIF
   50   CONTINUE
        STOP 'LOAD: Array overflow'
   55 CONTINUE
      NEL = NEX
C
   56 IF(I.EQ.1) THEN
C----- coordinate file has "999.0 999.0" at the end ...
       NEL = IEL-1
      ELSE
C----- coordinate file has no ending line
       NEL = IEL
       NTMP(IEL) = I-1
      ENDIF
C
C
      WRITE(*,3010) NEL
      CALL ASKI('Enter element to be replaced by current airfoil^',IEL)
C
      IF(IEL.LT.1 .OR. IEL.GT.NEL+1) THEN
       WRITE(*,*) 'Element number inappropriate.  Airfoil not written.'
       CLOSE(LU)
       RETURN
      ELSE IF(IEL.EQ.NEL+1) THEN
       NEL = NEL+1
      ENDIF
C
C
      NTMP(IEL) = N
      DO 70 I = 1, NTMP(IEL)
        IF(LCLOCK) THEN
C------- write out in clockwise order (reversed from internal XFOIL order)
         IDIR = NTMP(IEL) - I + 1
        ELSE
C------- write out in counterclockwise order (same as internal XFOIL order)
         IDIR = I
        ENDIF
        XTMP(I,IEL) = X(IDIR)
        YTMP(I,IEL) = Y(IDIR)
 70   CONTINUE
C
C
      REWIND(LU)
C
C---- write first 2 lines of MSES format coordinate file
      WRITE(LU,1000) NAME1(1:NN1)
      WRITE(LU,1000) ISPARS1(1:NI1)
C
      DO 80 IEL=1, NEL
        DO 805 I=1, NTMP(IEL)
          WRITE(LU,1100) XTMP(I,IEL),YTMP(I,IEL)
  805   CONTINUE
        IF(IEL.LT.NEL) WRITE(LU,*) ' 999.0  999.0'
   80 CONTINUE
C
      CLOSE(LU)
      RETURN
C
 9005 WRITE(*,*) 'Old file OPEN error.  Airfoil not saved.'
      RETURN
C
 9010 WRITE(*,*) 'Old file READ error.  Airfoil not saved.'
      CLOSE(LU)
      RETURN
C
 1000 FORMAT(A)
 1100 FORMAT(1X,2G15.7)
 3010 FORMAT(/' Specified multielement airfoil has',I2,' elements.')
      END ! MSAVE


 
      SUBROUTINE ROTATE(X,Y,N,ALFA)
      DIMENSION X(N), Y(N)
C
      SA = SIN(ALFA)
      CA = COS(ALFA)
CCC      XOFF = 0.25*(1.0-CA)
CCC      YOFF = 0.25*SA
      XOFF = 0.
      YOFF = 0.
      DO 8 I=1, N
        XT = X(I)
        YT = Y(I)
        X(I) = CA*XT + SA*YT + XOFF
        Y(I) = CA*YT - SA*XT + YOFF
    8 CONTINUE
C
      RETURN
      END


      SUBROUTINE NACA(IDES1)
      INCLUDE 'XFOIL.INC'
C
C---- number of points per side
      NSIDE = IQX/3
C
      IF(IDES1 .LE. 0) THEN
       CALL ASKI('Enter NACA 4 or 5-digit airfoil designation^',IDES)
      ELSE
       IDES = IDES1
      ENDIF
C
      ITYPE = 0
      IF(IDES.LE.25099) ITYPE = 5
      IF(IDES.LE.9999 ) ITYPE = 4
C
      IF(ITYPE.EQ.0) THEN
       WRITE(*,*) 'This designation not implemented.'
       RETURN
      ENDIF
C
      IF(ITYPE.EQ.4) CALL NACA4(IDES,W1,W2,W3,NSIDE,XB,YB,NB,NAME)
      IF(ITYPE.EQ.5) CALL NACA5(IDES,W1,W2,W3,NSIDE,XB,YB,NB,NAME)
      CALL STRIP(NAME,NNAME)
C
C---- see if routines didn't recognize designator
      IF(IDES.EQ.0) RETURN
C
      LCLOCK = .FALSE.
C
      XBF = 0.0
      YBF = 0.0
      LBFLAP = .FALSE.
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB, W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      WRITE(*,1200) NB
 1200 FORMAT(/' Buffer airfoil set using', I4,' points')
C
C---- set paneling
      CALL PANGEN(.TRUE.)
ccc      CALL PANPLT
C
      RETURN
      END ! NACA


      SUBROUTINE PANGEN(SHOPAR)
C---------------------------------------------------
C     Set paneling distribution from buffer airfoil
C     geometry, thus creating current airfoil.
C 
C     If REFINE=True, bunch points at x=XSREF on
C     top side and at x=XPREF on bottom side
C     by setting a fictitious local curvature of
C     CTRRAT*(LE curvature) there.
C---------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL SHOPAR
C
      IF(NB.LT.2) THEN
       WRITE(*,*) 'PANGEN: Buffer airfoil not available.'
       N = 0
       RETURN
      ENDIF
C
C---- Number of temporary nodes for panel distribution calculation
C       exceeds the specified panel number by factor of IPFAC.
      IPFAC = 3
      IPFAC = 5
C
C---- number of airfoil panel points
      N = NPAN
C
cC---- number of wake points
c      NW = NPAN/8 + 2
c      IF(NW.GT.IWX) THEN
c       WRITE(*,*)
c     &  'Array size (IWX) too small.  Last wake point index reduced.'
c       NW = IWX
c      ENDIF
C
C---- set arc length spline parameter
      CALL SCALC(XB,YB,SB,NB)
C
C---- spline raw airfoil coordinates
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
C---- normalizing length (~ chord)
      SBREF = 0.5*(SB(NB)-SB(1))
C
C---- set up curvature array
      DO I = 1, NB
        W5(I) = ABS( CURV(SB(I),XB,XBP,YB,YBP,SB,NB) ) * SBREF
      ENDDO
C
C---- locate LE point arc length value and the normalized curvature there
      CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
      CVLE = ABS( CURV(SBLE,XB,XBP,YB,YBP,SB,NB) ) * SBREF
C
C---- check for doubled point (sharp corner) at LE
      IBLE = 0
      DO I = 1, NB-1
        IF(SBLE.EQ.SB(I) .AND. SBLE.EQ.SB(I+1)) THEN
         IBLE = I
         WRITE(*,*)
         WRITE(*,*) 'Sharp leading edge'
         GO TO 21
        ENDIF
      ENDDO
 21   CONTINUE
C
C---- set LE, TE points
      XBLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YBLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XBTE = 0.5*(XB(1)+XB(NB))
      YBTE = 0.5*(YB(1)+YB(NB))
      CHBSQ = (XBTE-XBLE)**2 + (YBTE-YBLE)**2
C
C---- set average curvature over 2*NK+1 points within Rcurv of LE point
      NK = 3
      CVSUM = 0.
      DO K = -NK, NK
        FRAC = FLOAT(K)/FLOAT(NK)
        SBK = SBLE + FRAC*SBREF/MAX(CVLE,20.0)
        CVK = ABS( CURV(SBK,XB,XBP,YB,YBP,SB,NB) ) * SBREF
        CVSUM = CVSUM + CVK
      ENDDO
      CVAVG = CVSUM/FLOAT(2*NK+1)
C
C---- dummy curvature for sharp LE
      IF(IBLE.NE.0) CVAVG = 10.0
C
C---- set curvature attraction coefficient actually used
      CC = 6.0 * CVPAR
C
C---- set artificial curvature at TE to bunch panels there
      CVTE = CVAVG * CTERAT
      W5(1)  = CVTE
      W5(NB) = CVTE
C
C
C**** smooth curvature array for smoother panel size distribution  ****
C
CCC      CALL ASKR('Enter curvature smoothing length/c^',SMOOL)
CCC      SMOOL = 0.010
C
C---- set smoothing length = 1 / averaged LE curvature, but 
C-    no more than 5% of chord and no less than 1/4 average panel spacing
      SMOOL = MAX( 1.0/MAX(CVAVG,20.0) , 0.25 /FLOAT(NPAN/2) )
C
      SMOOSQ = (SMOOL*SBREF) ** 2
C
C---- set up tri-diagonal system for smoothed curvatures
      W2(1) = 1.0
      W3(1) = 0.0
      DO I=2, NB-1
        DSM = SB(I) - SB(I-1)
        DSP = SB(I+1) - SB(I)
        DSO = 0.5*(SB(I+1) - SB(I-1))
C
        IF(DSM.EQ.0.0 .OR. DSP.EQ.0.0) THEN
C------- leave curvature at corner point unchanged
         W1(I) = 0.0
         W2(I) = 1.0
         W3(I) = 0.0
        ELSE
         W1(I) =  SMOOSQ * (         - 1.0/DSM) / DSO
         W2(I) =  SMOOSQ * ( 1.0/DSP + 1.0/DSM) / DSO  +  1.0
         W3(I) =  SMOOSQ * (-1.0/DSP          ) / DSO
        ENDIF
      ENDDO
C
      W1(NB) = 0.0
      W2(NB) = 1.0
C
C---- fix curvature at LE point by modifying equations adjacent to LE
      DO I=2, NB-1
        IF(SB(I).EQ.SBLE .OR. I.EQ.IBLE .OR. I.EQ.IBLE+1) THEN
C------- if node falls right on LE point, fix curvature there
         W1(I) = 0.
         W2(I) = 1.0
         W3(I) = 0.
         W5(I) = CVLE
        ELSE IF(SB(I-1).LT.SBLE .AND. SB(I).GT.SBLE) THEN
C------- modify equation at node just before LE point
         DSM = SB(I-1) - SB(I-2)
         DSP = SBLE    - SB(I-1)
         DSO = 0.5*(SBLE - SB(I-2))
C
         W1(I-1) =  SMOOSQ * (         - 1.0/DSM) / DSO
         W2(I-1) =  SMOOSQ * ( 1.0/DSP + 1.0/DSM) / DSO  +  1.0
         W3(I-1) =  0.
         W5(I-1) = W5(I-1) + SMOOSQ*CVLE/(DSP*DSO)
C
C------- modify equation at node just after LE point
         DSM = SB(I) - SBLE
         DSP = SB(I+1) - SB(I)
         DSO = 0.5*(SB(I+1) - SBLE)
         W1(I) =  0.
         W2(I) =  SMOOSQ * ( 1.0/DSP + 1.0/DSM) / DSO  +  1.0
         W3(I) =  SMOOSQ * (-1.0/DSP          ) / DSO
         W5(I) = W5(I) + SMOOSQ*CVLE/(DSM*DSO)
C
         GO TO 51
        ENDIF
      ENDDO
   51 CONTINUE
C
C---- set artificial curvature at bunching points and fix it there
      DO I=2, NB-1
C------ chord-based x/c coordinate
        XOC = (  (XB(I)-XBLE)*(XBTE-XBLE)
     &         + (YB(I)-YBLE)*(YBTE-YBLE) ) / CHBSQ
C
        IF(SB(I).LT.SBLE) THEN
C------- check if top side point is in refinement area
         IF(XOC.GT.XSREF1 .AND. XOC.LT.XSREF2) THEN
          W1(I) = 0.
          W2(I) = 1.0
          W3(I) = 0.
          W5(I) = CVLE*CTRRAT
         ENDIF
        ELSE
C------- check if bottom side point is in refinement area
         IF(XOC.GT.XPREF1 .AND. XOC.LT.XPREF2) THEN
          W1(I) = 0.
          W2(I) = 1.0
          W3(I) = 0.
          W5(I) = CVLE*CTRRAT
         ENDIF
        ENDIF
      ENDDO
C
C---- solve for smoothed curvature array W5
      IF(IBLE.EQ.0) THEN
       CALL TRISOL(W2,W1,W3,W5,NB)
      ELSE
       I = 1
       CALL TRISOL(W2(I),W1(I),W3(I),W5(I),IBLE)
       I = IBLE+1
       CALL TRISOL(W2(I),W1(I),W3(I),W5(I),NB-IBLE)
      ENDIF
C
C---- find max curvature
      CVMAX = 0.
      DO I=1, NB
        CVMAX = MAX( CVMAX , ABS(W5(I)) )
      ENDDO
C
C---- normalize curvature array
      DO I=1, NB
        W5(I) = W5(I) / CVMAX
      ENDDO
C
C---- spline curvature array
      CALL SEGSPL(W5,W6,SB,NB)
C
C---- Set initial guess for node positions uniform in s.
C     More nodes than specified (by factor of IPFAC) are 
C     temporarily used  for more reliable convergence.
      NN = IPFAC*(N-1)+1
C
C---- ratio of lengths of panel at TE to one away from the TE
      RDSTE = 0.667
      RTF = (RDSTE-1.0)*2.0 + 1.0
C
      IF(IBLE.EQ.0) THEN
C
       DSAVG = (SB(NB)-SB(1))/(FLOAT(NN-3) + 2.0*RTF)
       SNEW(1) = SB(1)
       DO I=2, NN-1
         SNEW(I) = SB(1) + DSAVG * (FLOAT(I-2) + RTF)
       ENDDO
       SNEW(NN) = SB(NB)
C
      ELSE
C
       NFRAC1 = (N * IBLE) / NB
C
       NN1 = IPFAC*(NFRAC1-1)+1
       DSAVG1 = (SBLE-SB(1))/(FLOAT(NN1-2) + RTF)
       SNEW(1) = SB(1)
       DO I=2, NN1
         SNEW(I) = SB(1) + DSAVG1 * (FLOAT(I-2) + RTF)
       ENDDO
C
       NN2 = NN - NN1 + 1
       DSAVG2 = (SB(NB)-SBLE)/(FLOAT(NN2-2) + RTF)
       DO I=2, NN2-1
         SNEW(I-1+NN1) = SBLE + DSAVG2 * (FLOAT(I-2) + RTF)
       ENDDO
       SNEW(NN) = SB(NB)
C
      ENDIF
C
C---- Newton iteration loop for new node positions
      DO 10 ITER=1, 20
C
C------ set up tri-diagonal system for node position deltas
        CV1  = SEVAL(SNEW(1),W5,W6,SB,NB)
        CV2  = SEVAL(SNEW(2),W5,W6,SB,NB)
        CVS1 = DEVAL(SNEW(1),W5,W6,SB,NB)
        CVS2 = DEVAL(SNEW(2),W5,W6,SB,NB)
C
        CAVM = SQRT(CV1**2 + CV2**2)
        IF(CAVM .EQ. 0.0) THEN
          CAVM_S1 = 0.
          CAVM_S2 = 0.
        ELSE
          CAVM_S1 = CVS1 * CV1/CAVM
          CAVM_S2 = CVS2 * CV2/CAVM
        ENDIF
C
        DO 110 I=2, NN-1
          DSM = SNEW(I) - SNEW(I-1)
          DSP = SNEW(I) - SNEW(I+1)
          CV3  = SEVAL(SNEW(I+1),W5,W6,SB,NB)
          CVS3 = DEVAL(SNEW(I+1),W5,W6,SB,NB)
C
          CAVP = SQRT(CV3**2 + CV2**2)
          IF(CAVP .EQ. 0.0) THEN
            CAVP_S2 = 0.
            CAVP_S3 = 0.
          ELSE
            CAVP_S2 = CVS2 * CV2/CAVP
            CAVP_S3 = CVS3 * CV3/CAVP
          ENDIF
C
          FM = CC*CAVM + 1.0
          FP = CC*CAVP + 1.0
C
          REZ = DSP*FP + DSM*FM
C
C-------- lower, main, and upper diagonals
          W1(I) =      -FM  +  CC*               DSM*CAVM_S1
          W2(I) =  FP + FM  +  CC*(DSP*CAVP_S2 + DSM*CAVM_S2)
          W3(I) = -FP       +  CC* DSP*CAVP_S3
C
C-------- residual, requiring that
C         (1 + C*curv)*deltaS is equal on both sides of node i
          W4(I) = -REZ
C
          CV1 = CV2
          CV2 = CV3
          CVS1 = CVS2
          CVS2 = CVS3
          CAVM    = CAVP
          CAVM_S1 = CAVP_S2
          CAVM_S2 = CAVP_S3
  110   CONTINUE
C
C------ fix endpoints (at TE)
        W2(1) = 1.0
        W3(1) = 0.0
        W4(1) = 0.0
        W1(NN) = 0.0
        W2(NN) = 1.0
        W4(NN) = 0.0
C
        IF(RTF .NE. 1.0) THEN
C------- fudge equations adjacent to TE to get TE panel length ratio RTF
C
         I = 2
         W4(I) = -((SNEW(I) - SNEW(I-1)) + RTF*(SNEW(I) - SNEW(I+1)))
         W1(I) = -1.0
         W2(I) =  1.0 + RTF
         W3(I) =      - RTF
C
         I = NN-1
         W4(I) = -((SNEW(I) - SNEW(I+1)) + RTF*(SNEW(I) - SNEW(I-1)))
         W3(I) = -1.0
         W2(I) =  1.0 + RTF
         W1(I) =      - RTF
        ENDIF
C
C
C------ fix sharp LE point
        IF(IBLE.NE.0) THEN
         I = NN1
         W1(I) = 0.0
         W2(I) = 1.0
         W3(I) = 0.0
         W4(I) = SBLE - SNEW(I)
        ENDIF
C
C------ solve for changes W4 in node position arc length values
        CALL TRISOL(W2,W1,W3,W4,NN)
C
C------ find under-relaxation factor to keep nodes from changing order
        RLX = 1.0
        DMAX = 0.0
        DO I=1, NN-1
          DS  = SNEW(I+1) - SNEW(I)
          DDS = W4(I+1) - W4(I)
          DSRAT = 1.0 + RLX*DDS/DS
          IF(DSRAT.GT.4.0) RLX = (4.0-1.0)*DS/DDS
          IF(DSRAT.LT.0.2) RLX = (0.2-1.0)*DS/DDS
          DMAX = MAX(ABS(W4(I)),DMAX)
        ENDDO
C
C------ update node position
        DO I=2, NN-1
          SNEW(I) = SNEW(I) + RLX*W4(I)
        ENDDO
C
CCC        IF(RLX.EQ.1.0) WRITE(*,*) DMAX
CCC        IF(RLX.NE.1.0) WRITE(*,*) DMAX,'    RLX =',RLX
        IF(ABS(DMAX).LT.1.E-3) GO TO 11
   10 CONTINUE
      WRITE(*,*) 'Paneling convergence failed.  Continuing anyway...'
C
   11 CONTINUE
C
C---- set new panel node coordinates
      DO I=1, N
        IND = IPFAC*(I-1) + 1
        S(I) = SNEW(IND)
        X(I) = SEVAL(SNEW(IND),XB,XBP,SB,NB)
        Y(I) = SEVAL(SNEW(IND),YB,YBP,SB,NB)
      ENDDO
C
C
C---- go over buffer airfoil again, checking for corners (double points)
      NCORN = 0
      DO 25 IB=1, NB-1
        IF(SB(IB) .EQ. SB(IB+1)) THEN
C------- found one !
C
         NCORN = NCORN+1
         XBCORN = XB(IB)
         YBCORN = YB(IB)
         SBCORN = SB(IB)
C
C------- find current-airfoil panel which contains corner
         DO 252 I=1, N
C
C--------- keep stepping until first node past corner
           IF(S(I) .LE. SBCORN) GO TO 252
C
C---------- move remainder of panel nodes to make room for additional node
            DO 2522 J=N, I, -1
              X(J+1) = X(J)
              Y(J+1) = Y(J)
              S(J+1) = S(J)
 2522       CONTINUE
            N = N+1
C
            IF(N .GT. IQX-1)
     &       STOP 'PANEL: Too many panels. Increase IQX in XFOIL.INC'
C
            X(I) = XBCORN
            Y(I) = YBCORN
            S(I) = SBCORN
C
C---------- shift nodes adjacent to corner to keep panel sizes comparable
            IF(I-2 .GE. 1) THEN
             S(I-1) = 0.5*(S(I) + S(I-2))
             X(I-1) = SEVAL(S(I-1),XB,XBP,SB,NB)
             Y(I-1) = SEVAL(S(I-1),YB,YBP,SB,NB)
            ENDIF
C
            IF(I+2 .LE. N) THEN
             S(I+1) = 0.5*(S(I) + S(I+2))
             X(I+1) = SEVAL(S(I+1),XB,XBP,SB,NB)
             Y(I+1) = SEVAL(S(I+1),YB,YBP,SB,NB)
            ENDIF
C
C---------- go on to next input geometry point to check for corner
            GO TO 25
C
  252    CONTINUE
        ENDIF
   25 CONTINUE
C
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
C
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD  = SQRT( (XTE-XLE)**2 + (YTE-YLE)**2 )
C
C---- calculate panel size ratios (user info)
      DSMIN =  1000.0
      DSMAX = -1000.0
      DO 40 I=1, N-1
        DS = S(I+1)-S(I)
        IF(DS .EQ. 0.0) GO TO 40
          DSMIN = MIN(DSMIN,DS)
          DSMAX = MAX(DSMAX,DS)
   40 CONTINUE
C
      DSMIN = DSMIN*FLOAT(N-1)/S(N)
      DSMAX = DSMAX*FLOAT(N-1)/S(N)
ccc      WRITE(*,*) 'DSmin/DSavg = ',DSMIN,'     DSmax/DSavg = ',DSMAX
C
C---- set various flags for new airfoil
      LGAMU = .FALSE.
      LQINU = .FALSE.
      LWAKE = .FALSE.
      LQAIJ = .FALSE.
      LADIJ = .FALSE.
      LWDIJ = .FALSE.
      LIPAN = .FALSE.
      LBLINI = .FALSE.
      LVCONV = .FALSE.
      LSCINI = .FALSE.
      LQSPEC = .FALSE.
      LGSAME = .FALSE.
C
      IF(LBFLAP) THEN
       XOF = XBF
       YOF = YBF
       LFLAP = .TRUE.
      ENDIF
C
C---- determine if TE is blunt or sharp, calculate TE geometry parameters
      CALL TECALC
C
C---- calculate normal vectors
      CALL NCALC(X,Y,S,N,NX,NY)
C
C---- calculate panel angles for panel routines
      CALL APCALC
C
      IF(SHARP) THEN
       WRITE(*,1090) 'Sharp trailing edge'
      ELSE
       GAP = SQRT((X(1)-X(N))**2 + (Y(1)-Y(N))**2)
       WRITE(*,1090) 'Blunt trailing edge.  Gap =', GAP
      ENDIF
 1090 FORMAT(/1X,A,F9.5)
C
      IF(SHOPAR) WRITE(*,1100) NPAN, CVPAR, CTERAT, CTRRAT,
     &                         XSREF1, XSREF2, XPREF1, XPREF2
 1100 FORMAT(/' Paneling parameters used...'
     &       /'   Number of panel nodes      ' , I4
     &       /'   Panel bunching parameter   ' , F6.3
     &       /'   TE/LE panel density ratio  ' , F6.3
     &       /'   Refined-area/LE panel density ratio   ' , F6.3
     &       /'   Top    side refined area x/c limits ' , 2F6.3
     &       /'   Bottom side refined area x/c limits ' , 2F6.3)
C
      RETURN
      END ! PANGEN



      SUBROUTINE GETPAN
      INCLUDE 'XFOIL.INC'
      LOGICAL LCHANGE
      CHARACTER*4 VAR
      CHARACTER*128 COMARG
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
      IF(NB.LE.1) THEN
       WRITE(*,*) 'GETPAN: Buffer airfoil not available.'
       RETURN
      ENDIF
C
 5    CONTINUE
      IF(N.LE.1) THEN
       WRITE(*,*) 'No current airfoil to plot'
      ELSE
       CALL PANPLT
      ENDIF
      LCHANGE = .FALSE.
C
   10 WRITE(*,1000) NPAN, CVPAR, CTERAT, CTRRAT,
     &              XSREF1, XSREF2, XPREF1, XPREF2
 1000 FORMAT(
     & /'    Present paneling parameters...'
     & /'  N  i   Number of panel nodes      ' , I4
     & /'  P  r   Panel bunching parameter   ' , F6.3
     & /'  T  r   TE/LE panel density ratio  ' , F6.3
     & /'  R  r   Refined area/LE  panel density ratio  ' , F6.3
     & /'  XT rr  Top    side refined area x/c limits   ' , 2F6.3
     & /'  XB rr  Bottom side refined area x/c limits   ' , 2F6.3
     & /'  Z oom'
     & /'  U nzoom'
     & /'  H ardcopy' )
C
   12 CALL ASKC('Change what ? (<cr> if nothing else)^',VAR,COMARG)
C
      IF(VAR.EQ.'Z   ') THEN
        CALL USETZOOM(.TRUE.,.TRUE.)
        CALL REPLOT(IDEV)
        GO TO 12
      ENDIF
C
      IF(VAR.EQ.'U   ') THEN
        CALL CLRZOOM
        CALL REPLOT(IDEV)
        GO TO 12
      ENDIF
C
      IF(VAR.EQ.'H   ') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVRP)
       GO TO 12
      ENDIF
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
      IF     (VAR.EQ.'    ') THEN
C
        IF(LCHANGE) THEN
C
C-------- set new panel distribution, and display max panel corner angle
          CALL PANGEN(.FALSE.)
          IF(N.GT.0) CALL CANG(X,Y,N,1,IMAX,AMAX)
C
C-------- go back to paneling menu
          GO TO 5
        ENDIF
C
        CALL CLRZOOM
        RETURN
C
      ELSE IF(VAR.EQ.'N   ' .OR. VAR.EQ.'n   ') THEN
C
        IF(NINPUT.GE.1) THEN
         NPAN = IINPUT(1)
        ELSE
         CALL ASKI('Enter number of panel nodes^',NPAN)
        ENDIF
        IF(NPAN .GT. IQX-6) THEN
          NPAN = IQX - 6
          WRITE(*,1200) NPAN
 1200     FORMAT(1X,' Number of panel nodes reduced to array limit:',I4)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE IF(VAR.EQ.'P   ' .OR. VAR.EQ.'p   ') THEN
C
        IF(NINPUT.GE.1) THEN
         CVPAR = RINPUT(1)
        ELSE
         CALL ASKR('Enter panel bunching parameter (0 to ~1)^',CVPAR)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE IF(VAR.EQ.'T   ' .OR. VAR.EQ.'t   ') THEN
C
        IF(NINPUT.GE.1) THEN
         CTERAT = RINPUT(1)
        ELSE
         CALL ASKR('Enter TE/LE panel density ratio^',CTERAT)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE IF(VAR.EQ.'R   ' .OR. VAR.EQ.'r   ') THEN
C
        IF(NINPUT.GE.1) THEN
         CTRRAT = RINPUT(1)
        ELSE
         CALL ASKR('Enter refined-area panel density ratio^',CTRRAT)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE IF(VAR.EQ.'XT  ' .OR. VAR.EQ.'xt  ') THEN
C
        IF(NINPUT.GE.2) THEN
         XSREF1 = RINPUT(1)
         XSREF2 = RINPUT(2)
        ELSE
         CALL ASKR('Enter left   top   side refinement limit^',XSREF1)
         CALL ASKR('Enter right  top   side refinement limit^',XSREF2)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE IF(VAR.EQ.'XB  ' .OR. VAR.EQ.'xb  ') THEN
C
        IF(NINPUT.GE.2) THEN
         XPREF1 = RINPUT(1)
         XPREF2 = RINPUT(2)
        ELSE
         CALL ASKR('Enter left  bottom side refinement limit^',XPREF1)
         CALL ASKR('Enter right bottom side refinement limit^',XPREF2)
        ENDIF
        LCHANGE = .TRUE.
C
      ELSE
C
        WRITE(*,*)
        WRITE(*,*) '***  Input not recognized  ***'
        GO TO 10
C
      ENDIF
C
      GO TO 12
C
      END ! GETPAN


      SUBROUTINE TECALC
C-------------------------------------------
C     Calculates total and projected TE gap 
C     areas and TE panel strengths.
C-------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- set TE base vector and TE bisector components
      DXTE = X(1) - X(N)
      DYTE = Y(1) - Y(N)
      DXS = 0.5*(-XP(1) + XP(N))
      DYS = 0.5*(-YP(1) + YP(N))
C
C---- normal and streamwise projected TE gap areas
      ANTE = DXS*DYTE - DYS*DXTE
      ASTE = DXS*DXTE + DYS*DYTE
C
C---- total TE gap area
      DSTE = SQRT(DXTE**2 + DYTE**2)
C
      SHARP = DSTE .LT. 0.0001*CHORD
C
      IF(SHARP) THEN
       SCS = 1.0
       SDS = 0.0
      ELSE
       SCS = ANTE/DSTE
       SDS = ASTE/DSTE
      ENDIF
C
C---- TE panel source and vorticity strengths
      SIGTE = 0.5*(GAM(1) - GAM(N))*SCS
      GAMTE = -.5*(GAM(1) - GAM(N))*SDS
C
      SIGTE_A = 0.5*(GAM_A(1) - GAM_A(N))*SCS
      GAMTE_A = -.5*(GAM_A(1) - GAM_A(N))*SDS
C
      RETURN
      END ! TECALC
 


      SUBROUTINE INTE
C-----------------------------------------------------------
C     Interpolates two airfoils into an intermediate shape.
C     Extrapolation is also possible to a reasonable extent.
C-----------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*2 CAIR
      INTEGER NINT(2)
      REAL SINT(IBX,2),  
     &     XINT(IBX,2), XPINT(IBX,2),
     &     YINT(IBX,2), YPINT(IBX,2),
     &     SLEINT(2)
      CHARACTER*20 PROMPTN
      CHARACTER*48 NAMEINT(2)
      CHARACTER*80 ISPARST
C
      LU = 21
C
 1000 FORMAT(A)
C
      WRITE(*,1100) NAME
      DO IP=1, NPOL
        IF(NXYPOL(IP).GT.0) THEN
         WRITE(*,1200) IP, NAMEPOL(IP)
        ENDIF
      ENDDO
      IF    (NPOL.EQ.0) THEN
       PROMPTN = '" ( F C ):  '
       NPR = 12
      ELSEIF(NPOL.EQ.1) THEN
       PROMPTN = '" ( F C 1 ):  '
       NPR = 14
      ELSEIF(NPOL.EQ.2) THEN
       PROMPTN = '" ( F C 1 2 ):  '
       NPR = 16
      ELSE
       PROMPTN = '" ( F C 1 2.. ):  '
       NPR = 18
      ENDIF
C
 1100 FORMAT(/   '  F  disk file'
     &       /   '  C  current airfoil  ', A)
 1200 FORMAT( 1X,I2,'  polar airfoil    ', A)
C
 2100 FORMAT(/'  Select source of airfoil "',I1, A, $)
C
      DO 40 K = 1, 2
        IAIR = K - 1
 20     WRITE(*,2100) IAIR, PROMPTN(1:NPR)
        READ(*,1000) CAIR
C
        IF    (INDEX('Ff',CAIR(1:1)).NE.0) THEN
         CALL ASKS('Enter filename^',FNAME)
         CALL AREAD(LU,FNAME,IBX,
     &              XINT(1,K),YINT(1,K),NINT(K),
     &              NAMEINT(K),ISPARST,ITYPE,0)
         IF(ITYPE.EQ.0) RETURN
C
        ELSEIF(INDEX('Cc',CAIR(1:1)).NE.0) THEN
         IF(N.LE.1) THEN
          WRITE(*,*) 'No current airfoil available'
          GO TO 20
         ENDIF
C
         NINT(K) = N
         DO I = 1, N
           XINT(I,K) = X(I)
           YINT(I,K) = Y(I)
         ENDDO
         NAMEINT(K) = NAME
C
        ELSE
         READ(CAIR,*,ERR=90) IP
         IF(IP.LT.1 .OR. IP.GT.NPOL) THEN
          GO TO 90
         ELSEIF(NXYPOL(IP).LE.0) THEN
          GO TO 90
         ELSE
          NINT(K) = NXYPOL(IP)
          DO I = 1, NINT(K)
            XINT(I,K) = CPOLXY(I,1,IP)
            YINT(I,K) = CPOLXY(I,2,IP)
          ENDDO
         ENDIF
         NAMEINT(K) = NAMEPOL(IP)
C
        ENDIF
C
        CALL SCALC(XINT(1,K),YINT(1,K),SINT(1,K),NINT(K))
        CALL SEGSPLD(XINT(1,K),XPINT(1,K),SINT(1,K),NINT(K),-999.,-999.)
        CALL SEGSPLD(YINT(1,K),YPINT(1,K),SINT(1,K),NINT(K),-999.,-999.)
        CALL LEFIND(SLEINT(K),
     &              XINT(1,K),XPINT(1,K),
     &              YINT(1,K),YPINT(1,K),SINT(1,K),NINT(K))
 40   CONTINUE
C
      WRITE(*,*) 
      WRITE(*,*) 'airfoil "0":  ', NAMEINT(1)
      WRITE(*,*) 'airfoil "1":  ', NAMEINT(2)
      FRAC = 0.5
      CALL ASKR('Specify interpolating fraction  0...1^',FRAC)
C
      CALL INTER(XINT(1,1),XPINT(1,1),
     &           YINT(1,1),YPINT(1,1),SINT(1,1),NINT(1),SLEINT(1),
     &           XINT(1,2),XPINT(1,2),
     &           YINT(1,2),YPINT(1,2),SINT(1,2),NINT(2),SLEINT(2),
     &           XB,YB,NB,FRAC)
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB, W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      CALL ASKS('Enter new airfoil name^',NAME)
      CALL STRIP(NAME,NNAME)
      WRITE(*,*)
      WRITE(*,*) 'Result has been placed in buffer airfoil'
      WRITE(*,*) 'Execute PCOP or PANE to set new current airfoil'
      RETURN
C
 90   CONTINUE
      WRITE(*,*)
      WRITE(*,*) 'Invalid response'
      RETURN
      END ! INTE


      SUBROUTINE INTX
C-----------------------------------------------------------
C     Interpolates two airfoils into an intermediate shape.
C     Extrapolation is also possible to a reasonable extent.
C-----------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*2 CAIR
      INTEGER NINT(2)
      REAL SINT(IBX,2),  
     &     XINT(IBX,2), XPINT(IBX,2),
     &     YINT(IBX,2), YPINT(IBX,2),
     &     SLEINT(2)
      CHARACTER*20 PROMPTN
      CHARACTER*48 NAMEINT(2)
      CHARACTER*80 ISPARST
C
      LU = 21
C
 1000 FORMAT(A)
C
      WRITE(*,1100) NAME
      DO IP=1, NPOL
        IF(NXYPOL(IP).GT.0) THEN
         WRITE(*,1200) IP, NAMEPOL(IP)
        ENDIF
      ENDDO
      IF    (NPOL.EQ.0) THEN
       PROMPTN = '" ( F C ):  '
       NPR = 12
      ELSEIF(NPOL.EQ.1) THEN
       PROMPTN = '" ( F C 1 ):  '
       NPR = 14
      ELSEIF(NPOL.EQ.2) THEN
       PROMPTN = '" ( F C 1 2 ):  '
       NPR = 16
      ELSE
       PROMPTN = '" ( F C 1 2.. ):  '
       NPR = 18
      ENDIF
C
 1100 FORMAT(/   '  F  disk file'
     &       /   '  C  current airfoil  ', A)
 1200 FORMAT( 1X,I2,'  polar airfoil    ', A)
C
 2100 FORMAT(/'  Select source of airfoil "',I1, A, $)
C
      DO 40 K = 1, 2
        IAIR = K - 1
 20     WRITE(*,2100) IAIR, PROMPTN(1:NPR)
        READ(*,1000,ERR=90,END=90) CAIR
C
        IF(CAIR .EQ. ' ') THEN
         GO TO 90
C
        ELSEIF(INDEX('Ff',CAIR(1:1)).NE.0) THEN
         CALL ASKS('Enter filename^',FNAME)
         CALL AREAD(LU,FNAME,IBX,
     &              XINT(1,K),YINT(1,K),NINT(K),
     &              NAMEINT(K),ISPARST,ITYPE,0)
         IF(ITYPE.EQ.0) RETURN
C
        ELSEIF(INDEX('Cc',CAIR(1:1)).NE.0) THEN
         IF(N.LE.1) THEN
          WRITE(*,*) 'No current airfoil available'
          GO TO 20
         ENDIF
C
         NINT(K) = N
         DO I = 1, N
           XINT(I,K) = X(I)
           YINT(I,K) = Y(I)
         ENDDO
         NAMEINT(K) = NAME
C
        ELSE
         READ(CAIR,*,ERR=90) IP
         IF(IP.LT.1 .OR. IP.GT.NPOL) THEN
          GO TO 90
         ELSEIF(NXYPOL(IP).LE.0) THEN
          GO TO 90
         ELSE
          NINT(K) = NXYPOL(IP)
          DO I = 1, N
            XINT(I,K) = CPOLXY(I,1,IP)
            YINT(I,K) = CPOLXY(I,2,IP)
          ENDDO
         ENDIF
         NAMEINT(K) = NAMEPOL(IP)
C
        ENDIF
C
        CALL SCALC(XINT(1,K),YINT(1,K),SINT(1,K),NINT(K))
        CALL SEGSPLD(XINT(1,K),XPINT(1,K),SINT(1,K),NINT(K),-999.,-999.)
        CALL SEGSPLD(YINT(1,K),YPINT(1,K),SINT(1,K),NINT(K),-999.,-999.)
        CALL LEFIND(SLEINT(K),
     &              XINT(1,K),XPINT(1,K),
     &              YINT(1,K),YPINT(1,K),SINT(1,K),NINT(K))
 40   CONTINUE
C
      WRITE(*,*) 
      WRITE(*,*) 'airfoil "0":  ', NAMEINT(1)
      WRITE(*,*) 'airfoil "1":  ', NAMEINT(2)
      FRAC = 0.5
      CALL ASKR('Specify interpolating fraction  0...1^',FRAC)
C
      CALL INTERX(XINT(1,1),XPINT(1,1),
     &           YINT(1,1),YPINT(1,1),SINT(1,1),NINT(1),SLEINT(1),
     &           XINT(1,2),XPINT(1,2),
     &           YINT(1,2),YPINT(1,2),SINT(1,2),NINT(2),SLEINT(2),
     &           XB,YB,NB,FRAC)
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB, W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      CALL ASKS('Enter new airfoil name^',NAME)
      CALL STRIP(NAME,NNAME)
      WRITE(*,*)
      WRITE(*,*) 'Result has been placed in buffer airfoil'
      WRITE(*,*) 'Execute PCOP or PANE to set new current airfoil'
      RETURN
C
 90   CONTINUE
      WRITE(*,*)
      WRITE(*,*) 'Invalid response.  No action taken.'
      RETURN
      END ! INTX




