C***********************************************************************
C    Module:  xqdes.f
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
      SUBROUTINE QDES
C------------------------------------------------------
C     Mixed-Inverse design routine. Based on the 
C     same panel formulation as basic analysis method.
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*4 COMAND, COMOLD
      LOGICAL LRECALC
C
      CHARACTER*128 COMARG, ARGOLD
      CHARACTER*1 CHKEY
C
      REAL XBOX(2), YBOX(2)
      REAL XSP(IBX), YSP(IBX,IPX), YSPD(IBX,IPX)
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR, LPLNEW
C
      EXTERNAL NEWPLOTQ
C
      SAVE COMOLD, ARGOLD
C
C---- statement function for compressible Karman-Tsien velocity
      QCOMP(G) = G*(1.0-TKLAM) / (1.0 - TKLAM*(G/QINF)**2)
C
C
      COMAND = '****'
      COMARG = ' '
      LRECALC = .FALSE.
C
      IF(N.EQ.0) THEN
       WRITE(*,*)
       WRITE(*,*) '***  No airfoil available  ***'
       RETURN
      ENDIF
C
      LSYM = .TRUE.
C
C---- number of sub-intervals for Qspec(s) plotting
      NTQSPL = 1
      IF(LQSLOP) NTQSPL = 8
C
C---- make sure a current solution exists
      CALL SPECAL
C
C---- see if current Qspec, if any, didn't come from Full-Inverse
      IF(NSP.NE.N) THEN
        LQSPEC = .FALSE.
        LIQSET = .FALSE.
      ENDIF
C
C---- set alpha, etc corresponding to Q
      ALGAM = ALFA
      CLGAM = CL
      CMGAM = CM
C
C---- set "old" speed distribution Q, arc length, and x/c,y/c arrays
      CHX = XTE - XLE
      CHY = YTE - YLE
      CHSQ = CHX**2 + CHY**2
      NSP = N
      DO I=1, NSP
        QGAMM(I) = GAM(I)
        SSPEC(I) = S(I)/S(N)
        XSPOC(I) = ((X(I)-XLE)*CHX + (Y(I)-YLE)*CHY)/CHSQ
        YSPOC(I) = ((Y(I)-YLE)*CHX - (X(I)-XLE)*CHY)/CHSQ
      ENDDO
      SSPLE = SLE/S(N)
C
      WRITE(*,1150) ALGAM/DTOR, CLGAM
 1150 FORMAT(/' Current Q operating condition:'
     &       /' alpha = ', F8.3, ' deg.      CL = ', F8.4 / )
C
      IF(.NOT.LQSPEC) THEN
C----- initialize Qspec to "old" solution and notify user
       NQSP = 1
       KQTARG = 1
       CALL GAMQSP(1)
       WRITE(*,1155)
       LQSPEC = .TRUE.
      ENDIF
C
C---- initialize blowup parameters and plot Qspec(s)
      CALL QPLINI(.TRUE.)
      CALL QSPLOT
C
C
C====================================================
C---- start of menu loop
 500  CONTINUE
      COMOLD = COMAND
      ARGOLD = COMARG
C
 501  CALL ASKC('.QDES^',COMAND,COMARG)
C
C--------------------------------------------------------
C---- process previous command ?
      IF(COMAND(1:1).EQ.'!') THEN
        IF(COMOLD.EQ.'****') THEN
          WRITE(*,*) 'Previous .QDES command not valid'
          GO TO 501
        ELSE
          COMAND = COMOLD
          COMARG = ARGOLD
          LRECALC = .TRUE.
        ENDIF
      ELSE
        LRECALC = .FALSE.
      ENDIF
C
      IF(COMAND.EQ.'    ') THEN
C----- just <return> was typed... clean up plotting and exit OPER
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       LQSYM = .FALSE.
       LQSPPL = .FALSE.
       CALL CLRZOOM
       RETURN
      ENDIF
C
C---- extract command line numeric arguments
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
C--------------------------------------------------------
      IF(COMAND.EQ.'?   ') THEN
       WRITE(*,1050)
 1050  FORMAT(
     &  /'   <cr>   Return to Top Level'
     & //'   QSET   Reset Qspec <== Q'
     & //'   Modi   Modify Qspec'
     &  /'   MARK   Mark off target segment'
     &  /'   SMOO   Smooth Qspec inside target segment'
     &  /'   SLOP   Toggle modified-Qspec slope matching flag'
     & //'   eXec i Execute mixed-inverse calculation'
     &  /'   REST   Restore geometry from buffer airfoil'
     &  /'   CPXX   CPxx endpoint constraint toggle'
     & //'   Visc   Qvis overlay toggle'
     &  /'   REFL   Reflected Qspec overlay toggle'
     & //'   Plot   Plot Qspec (line) and Q (symbols)'
     &  /'   Blow   Blowup plot region'
     &  /'   Rese   Reset plot scale and origin'
     &  /'   Wind     Plot window adjust via cursor and keys'
     & //'   SIZE r Change absolute plot-object size'
     &  /'  .ANNO   Annotate plot'
     &  /'   HARD   Hardcopy current plot')
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'Z   ') THEN
       CALL USETZOOM(.TRUE.,.TRUE.)
       CALL REPLOT(IDEV)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'U   ') THEN
       CALL CLRZOOM
       CALL REPLOT(IDEV)
C
C--------------------------------------------------------
C---- re-initialize Qspec to Q
      ELSEIF(COMAND.EQ.'QSET') THEN
       CALL GAMQSP(1)
       CALL QPLINI(.FALSE.)
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- toggle Qvis plotting flag
      ELSEIF(COMAND.EQ.'VISC' .OR.
     &       COMAND.EQ.'V   ') THEN
       LQVDES = .NOT.LQVDES
       IF(LQVDES) THEN
        WRITE(*,*) 'Qspec & Qvis will be plotted'
       ELSE
        WRITE(*,*) 'Only Qspec will be plotted'
        CALL QPLINI(.FALSE.)
       ENDIF
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- toggle reflected Qspec plotting flag
      ELSEIF(COMAND.EQ.'REFL') THEN
       LQREFL = .NOT.LQREFL
       IF(LQREFL) THEN
        WRITE(*,*) 'Reflected Qspec will be plotted'
       ELSE
        WRITE(*,*) 'Reflected Qspec will not be plotted'
        CALL QPLINI(.FALSE.)
       ENDIF
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- get target segment endpoints
      ELSEIF(COMAND.EQ.'MARK') THEN
       CALL IQSGET
      GO TO 500
C
C--------------------------------------------------------
C---- modify Qspec
      ELSEIF(COMAND.EQ.'MODI' .OR.
     &       COMAND.EQ.'M   '      ) THEN
C----- make sure there is a Qspec(s) plot on the screen
       IF(.NOT.LQSPPL) THEN
        CALL QPLINI(.FALSE.)
        CALL QSPLOT
       ENDIF
       CALL GETCOLOR(ICOL0)
C
C----- set up arrays for calling MODIFY
       IFRST = 1
       ILAST = NSP
       NSIDE = 1
       NLINE = NQSP
       DO I = 1, NSP
         ISP = NSP - I + 1
         XSP(ISP) = 1.0 - SSPEC(I)
         DO KQSP = 1, NQSP
           GCOMP = QCOMP(QSPEC(I,KQSP))/QINF
           YSP(ISP,KQSP) = QFAC*GCOMP
         ENDDO
       ENDDO
       DO KQSP = 1, NQSP
         CALL SEGSPL(YSP(1,KQSP),YSPD(1,KQSP),XSP,NSP)
       ENDDO
C
C----- get the user's modifying input
       XBOX(1) = XMARG
       XBOX(2) = XPAGE-XMARG
       YBOX(1) = YMARG
       YBOX(2) = YPAGE-YMARG
       CALL MODIFY(IBX,IFRST,ILAST,NSIDE,NLINE,
     &             XSP,YSP,YSPD, LQSLOP,
     &             ISP1,ISP2,ISMOD,KQSP,
     &             XBOX,YBOX, XBOX,YBOX,SIZE,
     &             XOFF,YOFF,XSF,YSF, 'RED','RED',
     &             NEWPLOTQ )
C
C----- put modified info back into global arrays
       IQMOD2 = NSP - ISP1 + 1
       IQMOD1 = NSP - ISP2 + 1
       DO I=1, NSP
         ISP = NSP - I + 1
         QSCOM =  QINF*YSP(ISP,KQSP)/QFAC
         QSPEC(I,KQSP) = QINCOM(QSCOM,QINF,TKLAM)
       ENDDO
C
C----- display new splined Qspec(s)
       CALL SPLQSP(KQSP)
       CALL NEWCOLORNAME('MAGENTA')
       CALL QSPPLT(IQMOD1,IQMOD2,KQSP,NTQSPL)
       CALL NEWCOLOR(ICOL0)
C
C----- print forces associated with modified Qspec(s)
       CALL PLFLUSH
       CALL CLCALC(N,X,Y,QSPEC(1,KQSP),W1,ALFA,MINF,QINF, XCMREF,YCMREF, 
     &             CLQSP(KQSP),CMQSP(KQSP),CDPQ, CLQ_ALF,CLQ_MSQ)
       WRITE(*,1200) CL,CM,CLQSP(KQSP),CMQSP(KQSP)
      GO TO 500
C
C--------------------------------------------------------
C---- smooth Qspec within target segment, or entire Qspec if not marked off
      ELSEIF(COMAND.EQ.'SMOO') THEN
       CALL GETCOLOR(ICOL0)
C
       KQSP = 1
       CALL SMOOQ(IQ1,IQ2,KQSP)
       CALL SPLQSP(KQSP)
C
       CALL NEWCOLORNAME('magenta')
       CALL QSPPLT(IQ1,IQ2,KQSP,NTQSPL)
       CALL NEWCOLOR(ICOL0)
       CALL PLFLUSH
       LQSPPL = .FALSE.
C
       CALL CLCALC(N,X,Y,QSPEC(1,KQSP),W1,ALFA,MINF,QINF, XCMREF,YCMREF, 
     &             CLQSP(KQSP),CMQSP(KQSP),CDPQ, CLQ_ALF,CLQ_MSQ)
        WRITE(*,1200) CL,CM,CLQSP(KQSP),CMQSP(KQSP)
      GO TO 500
C
C--------------------------------------------------------
C---- toggle Qspec endpoint slope matching
      ELSEIF(COMAND.EQ.'SLOP') THEN
       LQSLOP = .NOT.LQSLOP
       IF(LQSLOP) THEN
         WRITE(*,*)
     &    'Modified Qspec piece will be made tangent at endpoints'
       ELSE
         WRITE(*,*)
     &   'Modified Qspec piece will not be made tangent at endpoints'
       ENDIF
      GO TO 500
C
C--------------------------------------------------------
C---- hardcopy replot
      ELSEIF(COMAND.EQ.'HARD') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVRP)
      GO TO 500
C
C--------------------------------------------------------
C---- plot Qspec and Q distributions
      ELSEIF(COMAND.EQ.'PLOT' .OR.
     &       COMAND.EQ.'P   '      ) THEN
       CALL QPLINI(.FALSE.)
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- get blowup parameters
      ELSEIF(COMAND.EQ.'BLOW' .OR.
     &       COMAND.EQ.'B   '      ) THEN
       XWS = XWIND/SIZE
       YWS = YWIND/SIZE
       CALL OFFGET(XOFF,YOFF,XSF,YSF,XWS,YWS, .FALSE. , .TRUE. )
       CALL QPLINI(.FALSE.)
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- reset blowup parameters and replot
      ELSEIF(COMAND.EQ.'RESE' .OR.
     &       COMAND.EQ.'R   '      ) THEN
       CALL QPLINI(.TRUE.)
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'WIND' .OR.
     &       COMAND.EQ.'W   '      ) THEN
       XWS = XWIND/SIZE
       YWS = YWIND/SIZE
C
       WRITE(*,*) ' '
       WRITE(*,*) 'Type I,O,P to In,Out,Pan with cursor...'
C
 80    CALL QPLINI(.FALSE.)
       CALL QSPLOT
C
       CALL GETCURSORXY(XCRS,YCRS,CHKEY)
C
C----- do possible pan,zoom operations based on CHKEY
       CALL KEYOFF(XCRS,YCRS,CHKEY, XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
       IF(LPLNEW) THEN
        GO TO 80
       ENDIF
C
C--------------------------------------------------------
C---- annotate plot
      ELSEIF(COMAND.EQ.'ANNO') THEN
       IF(LPLOT) THEN
        CALL ANNOT(CH)
       ELSE
        WRITE(*,*) 'No active plot to annotate'
       ENDIF
      GO TO 500
C
C--------------------------------------------------------
C---- change plot size
      ELSEIF(COMAND.EQ.'SIZE') THEN
       IF(NINPUT.GE.1) THEN
        SIZE = RINPUT(1)
       ELSE
        WRITE(*,*) 'Current plot size =', SIZE
        CALL ASKR('Enter new plot size^',SIZE)
       ENDIF
C
       CALL QPLINI(.FALSE.)
       CALL QSPLOT
      GO TO 500
C
C--------------------------------------------------------
C---- toggle CPxx preservation constraints
      ELSEIF(COMAND.EQ.'CPXX') THEN
       LCPXX = .NOT.LCPXX
       IF(LCPXX) THEN
        WRITE(*,*) 'CPxx will be constrained'
       ELSE
        WRITE(*,*) 'CPxx will not be constrained'
       ENDIF
      GO TO 500
C
C--------------------------------------------------------
C---- set up for mixed-inverse calculation
      ELSEIF(COMAND.EQ.'EXEC' .OR.
     &       COMAND.EQ.'X   '      ) THEN
       IF(.NOT.LIQSET) THEN
        WRITE(*,*) '***  Must mark off target segment first  ***'
        GO TO 500
       ENDIF
C
C---- check if target segment includes stagnation point
       IST = 0
       DO I=IQ1, IQ2-1
         IF(QGAMM(I).GE.0.0 .AND. QGAMM(I+1).LT.0.0) IST = I
       ENDDO
C
       IF(IST.NE.0) THEN 
        WRITE(*,*)
        WRITE(*,*) 'Target segment cannot include ',
     &             'stagnation point in mixed-inverse.'
        GO TO 500
       ENDIF
C
       KQSP = 1
       CLSPEC = CLQSP(KQSP)
CCC      CALL ASKR('Enter specified CL^',CLSPEC)
C
C----- save current coordinates for restoration if requested
       DO I=1, N
         XB(I) = X(I)
         YB(I) = Y(I)
         SB(I) = S(I)
         XBP(I) = XP(I)
         YBP(I) = YP(I)
       ENDDO
       NB = N
       LGSAME = .TRUE.
C
       WRITE(*,*)
       WRITE(*,*) 'Current airfoil saved in buffer airfoil'
C
C----- execute mixed-inverse calculation
       IF(NINPUT.GE.1) THEN
        NITERQ = IINPUT(1)
       ELSE
        CALL ASKI('Enter max number of iterations^',NITERQ)
       ENDIF
C
       CALL MIXED(KQSP,NITERQ)
       ADEG = ALFA/DTOR
C
C----- spline new airfoil shape
       CALL SCALC(X,Y,S,N)
       CALL SPLIND(X,XP,S,N,-999.0,-999.0)
       CALL SPLIND(Y,YP,S,N,-999.0,-999.0)
       CALL NCALC(X,Y,S,N,NX,NY)
       CALL LEFIND(SLE,X,XP,Y,YP,S,N)
       XLE = SEVAL(SLE,X,XP,S,N)
       YLE = SEVAL(SLE,Y,YP,S,N)
       CHORD  = SQRT( (0.5*(X(1)+X(N)) - XLE)**2
     &              + (0.5*(Y(1)+Y(N)) - YLE)**2 )
       CALL TECALC
       CALL APCALC
C
       ALGAM = ALFA
C
       NSP = N
       DO I=1, N
         QGAMM(I) = GAM(I)
         SSPEC(I) = S(I)/S(N)
       ENDDO
       SSPLE = SLE/S(N)
C
C----- set inviscid surface speeds and calculate compressible Cp
       DO I=1, N
         QINV(I) = GAM(I)
       ENDDO
       CALL CPCALC(N,QINV,QINF,MINF,CPI)
C
C----- influence coefficients & other stuff is no longer valid for new airfoil
       LGAMU = .FALSE.
       LQINU = .FALSE.
       LWAKE = .FALSE.
       LQAIJ = .FALSE.
       LADIJ = .FALSE.
       LWDIJ = .FALSE.
       LIPAN = .FALSE.
       LVCONV = .FALSE.
       LSCINI = .FALSE.
CCC      LBLINI = .FALSE.
       LGSAME = .FALSE.
C
cc       CALL NAMMOD(NAME,1,1)
cc       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
C---- restore and spline old airfoil
      ELSEIF(COMAND.EQ.'REST') THEN
       DO I=1, N
         X(I) = XB(I)
         Y(I) = YB(I)
       ENDDO
       CALL SCALC(X,Y,S,N)
       CALL SPLIND(X,XP,S,N,-999.0,-999.0)
       CALL SPLIND(Y,YP,S,N,-999.0,-999.0)
       CALL NCALC(X,Y,S,N,NX,NY)
       CALL LEFIND(SLE,X,XP,Y,YP,S,N)
       XLE = SEVAL(SLE,X,XP,S,N)
       YLE = SEVAL(SLE,Y,YP,S,N)
       CHORD  = SQRT( (0.5*(X(1)+X(N)) - XLE)**2
     &              + (0.5*(Y(1)+Y(N)) - YLE)**2 )
       CALL TECALC
       CALL APCALC
       LGAMU = .FALSE.
       LQINU = .FALSE.
       LGSAME = .TRUE.
C
cc       CALL NAMMOD(NAME,-1,1)
cc       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
      ELSE
       WRITE(*,1100) COMAND
 1100  FORMAT(' Command ',A4,' not recognized.  Type a " ? " for list.')
C
       COMAND = '****'
      ENDIF
C
      GO TO 500
C
C....................................................
C
 1155 FORMAT(/' Qspec initialized to current Q.'/ )
 1200 FORMAT(/' Q    :   CL =',F11.6, '    CM =',F11.6
     &       /' Qspec:   CL =',F11.6, '    CM =',F11.6 )
      END


      SUBROUTINE NEWPLOTQ
      CALL QPLINI(.FALSE.)
      CALL QSPLOT
      RETURN
      END


      SUBROUTINE QPLINI(LDEF)
C----------------------------------------------
C     Sets up Qspec(s) plot.
C     If LDEF=t, sets default offsets.
C----------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LDEF
      LOGICAL LAIR
C
C---- number of x/c grid lines
      PARAMETER (NG=10,NQ=100)
      REAL SSPG(-NG:NG), SLPG(-NG:NG),
     &     XSPG(-NG:NG)
      REAL QSPG(-NQ:NQ)
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      INCLUDE 'XDES.INC'
C
C---- statement function for compressible Karman-Tsien velocity
      QCOMP(G) = G*(1.0-TKLAM) / (1.0 - TKLAM*(G/QINF)**2)
C
C
C---- make room for airfoil plot if complex-mapping routine is being used
      LAIR = NSP .EQ. NC1
C
C---- speed annotation increment
      DQANN  = 0.5
      DQANN2 = 0.1
C
C---- find max and min speeds for current Qgamm and Qspec
      QMIN = QGAMM(1)
      QMAX = QGAMM(1)
      DO 5 I=2, NSP
        QMIN = MIN(QMIN,QGAMM(I))
        QMAX = MAX(QMAX,QGAMM(I))
    5 CONTINUE
C
      DO 7 KQSP=1, NQSP
        DO 72 I=2, NSP
          QMIN = MIN(QMIN,QSPEC(I,KQSP))
          QMAX = MAX(QMAX,QSPEC(I,KQSP))
 72     CONTINUE
 7    CONTINUE
C
      QMIN = QCOMP(QMIN)/QINF
      QMAX = QCOMP(QMAX)/QINF
C
C---- round up to bounding annotations
      NMIN = INT(QMIN/DQANN) - 1
      NMAX = INT(QMAX/DQANN) + 1
C
      IF(LQREFL) THEN
C----- set limits so reflected Qspec(s) also fits on plot
       NMAX = MAX( ABS(NMIN) , ABS(NMAX) )
       NMIN = -NMAX
      ENDIF
C
      QMIN = DQANN*FLOAT(NMIN)
      QMAX = DQANN*FLOAT(NMAX)
C
C
C---- start new plot
      CALL PLTINI
C
C---- speed plotting scale factor
      QFAC = 1.0/(QMAX-QMIN)
C
C---- default offsets
      IF(LDEF) THEN
        XADD = 0.050
        YADD = 0.075
C
        XWMIN = MIN( XWIND - XMARG , XPAGE - 2.0*XMARG )
        YWMIN = MIN( YWIND - YMARG , YPAGE - 2.0*YMARG )
C
        XSF = (XWMIN/SIZE) / (1.0 + 2.0*XADD)
        YSF = (YWMIN/SIZE) / (1.0 + 2.0*YADD)
        CHQ = 0.7*CH * XSF
        XOFF = -XADD - 2.0*CHQ/XSF
        YOFF = -YADD + QMIN*QFAC
      ENDIF
C
      CALL SPLIND(XSPOC,W7,SSPEC,NSP,-999.0,-999.0)
      CALL SPLIND(YSPOC,W8,SSPEC,NSP,-999.0,-999.0)
C
      DO 11 IG=1, NG
        XOC = FLOAT(IG)/FLOAT(NG)
        SSP = SSPLE + (SSPEC(1)-SSPLE)*XOC
        CALL SINVRT(SSP,XOC,XSPOC,W7,SSPEC,NSP)
        SSPG(IG) = XMOD(1.0-SSP)
        XSPG(IG) = XOC
C
        XOC = 0.1*FLOAT(IG)/FLOAT(NG)
        SSP = SSPLE + (SSPEC(1)-SSPLE)*XOC
        CALL SINVRT(SSP,XOC,XSPOC,W7,SSPEC,NSP)
        SLPG(IG) = XMOD(1.0-SSP)
   11 CONTINUE
C
      XSPG(0) = 0.0
      SSPG(0) = XMOD(1.0-SSPLE)
      SLPG(0) = XMOD(1.0-SSPLE)
C
      DO 12 IG=-NG,-1
        XOC = FLOAT(-IG)/FLOAT(NG)
        SSP = SSPLE + (SSPEC(NSP)-SSPLE)*XOC
        CALL SINVRT(SSP,XOC,XSPOC,W7,SSPEC,NSP)
        SSPG(IG) = XMOD(1.0-SSP)
        XSPG(IG) = XOC
C
        XOC = 0.1*FLOAT(-IG)/FLOAT(NG)
        SSP = SSPLE + (SSPEC(NSP)-SSPLE)*XOC
        CALL SINVRT(SSP,XOC,XSPOC,W7,SSPEC,NSP)
        SLPG(IG) = XMOD(1.0-SSP)
   12 CONTINUE
C
C
C---- plot axes
      CALL NEWPEN(1)
      CALL PLOT(XMOD(0.0),YMOD(0.0),3)
      CALL PLOT(XMOD(1.0),YMOD(0.0),2)
      CALL PLOT(XMOD(0.0),YMOD(QFAC*QMIN),3)
      CALL PLOT(XMOD(0.0),YMOD(QFAC*QMAX),2)
      CALL PLOT(XMOD(1.0),YMOD(QFAC*QMIN),3)
      CALL PLOT(XMOD(1.0),YMOD(QFAC*QMAX),2)
C
C---- plot sonic lines if within range
      IF( QSTAR/QINF.LE.QMAX)
     &  CALL DASH(XMOD(0.0),XMOD(1.0),YMOD( QFAC*QSTAR/QINF))
      IF(-QSTAR/QINF.GE.QMIN)
     &  CALL DASH(XMOD(0.0),XMOD(1.0),YMOD(-QFAC*QSTAR/QINF))
C
C---- annotate axes
      DO 20 NT = NMIN, NMAX
        YPLT = QFAC*(QMAX-QMIN)*FLOAT(NT)/FLOAT(NMAX-NMIN)
ccc        IF(MOD(NT,2).EQ.0) THEN
         RNUM = DQANN*FLOAT(NT)
         CALL NEWPEN(2)
         XNUM = XMOD( 0.0)-3.5*CHQ 
         YNUM = YMOD(YPLT)-0.5*CHQ
         IF(RNUM.LT.0.0) XNUM = XNUM - CHQ
         CALL PLNUMB(XNUM,YNUM,CHQ,RNUM,0.0,1)
ccc        ENDIF
C
        IF(IABS(NT).LE.NQ) THEN
         QSPG(NT) = YMOD(YPLT)
        ELSE
         QSPG(NT) = YMOD(0.0)
        ENDIF
C
        CALL NEWPEN(1)
        CALL PLOT(XMOD(0.0)        ,YMOD(YPLT),3)
        CALL PLOT(XMOD(0.0)-0.3*CHQ,YMOD(YPLT),2)
        CALL PLOT(XMOD(1.0)        ,YMOD(YPLT),3)
        CALL PLOT(XMOD(1.0)+0.3*CHQ,YMOD(YPLT),2)
   20 CONTINUE
C
      XPLT = 0.5*(SSPG(NG-2)+SSPG(NG-3)) - 1.8*CHQ
      CALL PLCHAR(XPLT,YMOD(0.0)-3.0*CHQ,1.2*CHQ,'x/c',0.0,3)
C
      YPLT = QFAC*(QMAX-QMIN)*(FLOAT(NMAX)-1.5)/FLOAT(NMAX-NMIN)
      CALL PLCHAR(XMOD(0.0)-4.8*CHQ,YMOD(YPLT)-0.6*CHQ,
     &            1.2*CHQ,'q/V ',0.0,4)
      CALL PLMATH(XMOD(0.0)-4.8*CHQ,YMOD(YPLT)-0.6*CHQ,
     &            1.2*CHQ,'   &',0.0,4)
C
      INCR = MAX((2*NG)/20,1)
      DO 21 IG=-NG+INCR, NG-INCR, INCR
        CALL PLOT(SSPG(IG),QSPG(0)+0.20*CHQ,3)
        CALL PLOT(SSPG(IG),QSPG(0)-0.20*CHQ,2)
        CALL PLOT(SLPG(IG),QSPG(0)+0.15*CHQ,3)
        CALL PLOT(SLPG(IG),QSPG(0)-0.15*CHQ,2)
        IF    (IG.LT.0) THEN
         CALL PLNUMB(SSPG(IG)-0.9*CHQ,
     &               QSPG(0) +0.5*CHQ,0.7*CHQ,XSPG(IG),0.,1)
        ELSEIF(IG.GT.0) THEN
         CALL PLNUMB(SSPG(IG)-0.9*CHQ,
     &               QSPG(0) -1.3*CHQ,0.7*CHQ,XSPG(IG),0.,1)
        ENDIF
   21 CONTINUE
C
      INCR = MAX((2*NG)/4,1)
      DO 22 IG=-NG+INCR, NG-INCR, INCR
        CALL PLOT(SSPG(IG),QSPG(0)+0.40*CHQ,3)
        CALL PLOT(SSPG(IG),QSPG(0)-0.40*CHQ,2)
        CALL PLOT(SLPG(IG),QSPG(0)+0.30*CHQ,3)
        CALL PLOT(SLPG(IG),QSPG(0)-0.30*CHQ,2)
   22 CONTINUE
C
      INCR = MAX((2*NG)/2,1)
      DO 23 IG=-NG+INCR, NG-INCR, INCR
        CALL PLOT(SSPG(IG),QSPG(0)+0.80*CHQ,3)
        CALL PLOT(SSPG(IG),QSPG(0)-0.80*CHQ,2)
        CALL PLOT(SLPG(IG),QSPG(0)+0.60*CHQ,3)
        CALL PLOT(SLPG(IG),QSPG(0)-0.60*CHQ,2)
   23 CONTINUE
C
C
      IF(LQGRID) THEN
C---- grid lines in X and Q
        DO K=1, NG
          W1(K) = SSPG(K-NG) - SSPG(K-1-NG)
          W2(K) = SSPG(K)    - SSPG(K-1)
          W6(K) = SLPG(K-NG) - SLPG(K-1-NG)
          W7(K) = SLPG(K)    - SLPG(K-1)
        END DO
        DO K=1, -NMIN
          W3(K) = QSPG(K+NMIN) - QSPG(K-1+NMIN)
        END DO
        DO K=1, NMAX
          W4(K) = QSPG(K)      - QSPG(K-1)
        END DO
        CALL NEWPEN(1)
        CALL PLGRID(SSPG(-NG),QSPG(NMIN),1000+NG,W1,1000-NMIN,W3,LMASK2)
        CALL PLGRID(SSPG(0)  ,QSPG(0)   ,1000+NG,W2,1000+NMAX,W4,LMASK2)
C
C---- Intermediate fine grid lines in Q
        NFINE = IFIX(DQANN/DQANN2+0.5) + 1
        NMIN2 = NMIN*NFINE
        NMAX2 = NMAX*NFINE
        IF(NMIN2 .LT. -NQ .OR. NMAX2 .GT. NQ) THEN
         NFINE = 2
         NMIN2 = MAX( NMIN*NFINE , -NQ )
         NMAX2 = MIN( NMAX*NFINE ,  NQ )
        ENDIF

        DO NT = NMIN2, NMAX2
          YPLT = QFAC*(QMAX-QMIN)*FLOAT(NT)/FLOAT(NMAX2-NMIN2)
          QSPG(NT) = YMOD(YPLT)
        END DO
        DO K=1, -NMIN2
          W3(K) = QSPG(K+NMIN2) - QSPG(K-1+NMIN2)
        END DO
        DO K=1, NMAX2
          W4(K) = QSPG(K)      - QSPG(K-1)
        END DO
        CALL NEWPEN(1)
        CALL PLGRID(SSPG(-NG),QSPG(NMIN2),1000+NG,W1,
     &              1000-NMIN2,W3,LMASK1)
        CALL PLGRID(SSPG(0)  ,QSPG(0)   ,1000+NG,W2,
     &              1000+NMAX2,W4,LMASK1)
C
c        CALL PLGRID(SLPG(-NG),QSPG(NMIN),1000+NG,W6,1000-NMIN,W3,LMASK1)
c        CALL PLGRID(SLPG(0)  ,QSPG(0)   ,1000+NG,W7,1000+NMAX,W4,LMASK1)
      ENDIF
C
      CALL PLFLUSH
C
      RETURN
      END



      SUBROUTINE QSPLOT
C------------------------------------------------
C     Plots Q(s) and Qspec(s) distributions.
C------------------------------------------------
      INCLUDE 'XFOIL.INC'
      INCLUDE 'XDES.INC'
C
C---- statement function for compressible Karman-Tsien velocity
      QCOMP(G) = G*(1.0-TKLAM) / (1.0 - TKLAM*(G/QINF)**2)
C
C---- symbol height
      SHT = 0.4*CHQ
C
      CALL GETCOLOR(ICOL0)
C
      IF(LSYM) THEN
       IF(LIQSET) CALL NEWCOLORNAME('cyan')
       DO 50 I=1, NSP
         IF(LIQSET .AND. I.EQ.IQ1) CALL NEWCOLOR(ICOL0)
         XPLT = 1.0 - SSPEC(I)
         YPLT = QFAC*QCOMP(QGAMM(I))/QINF
         CALL PLSYMB(XMOD(XPLT),YMOD(YPLT),SHT,3,0.,0)
         IF(LIQSET .AND. I.EQ.IQ2) CALL NEWCOLORNAME('cyan')
   50  CONTINUE
       IF(LIQSET) CALL NEWCOLOR(ICOL0)
      ENDIF
C
      NTQSPL = 1
      IF(LQSLOP) NTQSPL = 8
C
C---- plot individual Qspec lines
      DO 60 KQSP=1, NQSP
        IF(LIQSET) THEN
         CALL NEWCOLORNAME('cyan')
         CALL QSPPLT(1,IQ1,KQSP,NTQSPL)
         CALL NEWCOLOR(ICOL0)
         CALL QSPPLT(IQ1,IQ2,KQSP,NTQSPL)
         CALL NEWCOLORNAME('cyan')
         CALL QSPPLT(IQ2,NSP,KQSP,NTQSPL)
         CALL NEWCOLOR(ICOL0)
        ELSE
         CALL QSPPLT(1,NSP,KQSP,NTQSPL)
        ENDIF
 60   CONTINUE
C
C
      IF(LQVDES) THEN
       CALL NEWCOLORNAME('orange')
       DO 65 I=2, N
         DSP = S(I) - S(I-1)
         DQV = QCOMP(QVIS(I)) - QCOMP(QVIS(I-1))
         SP1 = (S(I-1) + 0.25*DSP)/S(N)
         SP2 = (S(I)   - 0.25*DSP)/S(N)
         QV1 = QCOMP(QVIS(I-1)) + 0.25*DQV
         QV2 = QCOMP(QVIS(I)  ) - 0.25*DQV
         CALL PLOT(XMOD(1.0-SP1),YMOD(QFAC*QV1/QINF),3)
         CALL PLOT(XMOD(1.0-SP2),YMOD(QFAC*QV2/QINF),2)
   65  CONTINUE
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
      IF(LQREFL) THEN
       IF(LIQSET) CALL NEWCOLORNAME('cyan')
C
       KQSP = 1
C
C----- find stagnation point SSPEC value SSPST
       DO 70 ISTSP=1, NSP-1
         IF(QSPEC(ISTSP+1,KQSP).LT.0.0) GO TO 71
   70  CONTINUE
   71  DSSP = SSPEC(ISTSP+1)   - SSPEC(ISTSP)
       DQSP = QSPEC(ISTSP+1,KQSP) - QSPEC(ISTSP,KQSP)
       SSPST = SSPEC(ISTSP) - QSPEC(ISTSP,KQSP)*DSSP/DQSP
C
C----- plot reflected suction side QSPEC over pressure side QSPEC,
C-     fudging arc length SSPEC so stagnation points conside
       SPFUDG = (SSPEC(NSP) - SSPST) / (SSPST - SSPEC(1))
       DO 80 I=2, ISTSP
         DSP =  SSPEC(I) - SSPEC(I-1)
         DQS = QCOMP(QSPEC(I,KQSP)) - QCOMP(QSPEC(I-1,KQSP))
         SP1 = (SSPEC(I-1) + 0.35*DSP)*SPFUDG
         SP2 = (SSPEC(I)   - 0.35*DSP)*SPFUDG
         QS1 = QCOMP(QSPEC(I-1,KQSP)) + 0.35*DQS
         QS2 = QCOMP(QSPEC(I  ,KQSP)) - 0.35*DQS
         CALL PLOT(XMOD(SP1),YMOD(-QFAC*QS1/QINF),3)
         CALL PLOT(XMOD(SP2),YMOD(-QFAC*QS2/QINF),2)
   80  CONTINUE
C
C----- plot reflected pressure side QSPEC over suction side QSPEC,
C-     again fudging arc length SSPEC so stagnation points coincide
       SPFUDG = (SSPST - SSPEC(1)) / (SSPEC(NSP) - SSPST)
       DO 85 I=ISTSP+1, NSP
         DSP =  SSPEC(I) - SSPEC(I-1)
         DQS = QCOMP(QSPEC(I,KQSP)) - QCOMP(QSPEC(I-1,KQSP))
         SP1 = 1.0 - SSPST + (SSPEC(I-1) + 0.35*DSP - SSPST)*SPFUDG
         SP2 = 1.0 - SSPST + (SSPEC(I)   - 0.35*DSP - SSPST)*SPFUDG
         QS1 = QCOMP(QSPEC(I-1,KQSP)) + 0.35*DQS
         QS2 = QCOMP(QSPEC(I  ,KQSP)) - 0.35*DQS
         CALL PLOT(XMOD(SP1),YMOD(-QFAC*QS1/QINF),3)
         CALL PLOT(XMOD(SP2),YMOD(-QFAC*QS2/QINF),2)
   85  CONTINUE
C
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
C
      CALL PLFLUSH
      LQSPPL = .TRUE.
C
      IF(.NOT.LIQSET) RETURN
C
      KQSP = KQTARG
C
      CALL NEWCOLORNAME('cyan')
      YPLT1 = QFAC*QCOMP(QSPEC(IQ1,KQSP))/QINF
      YPLT2 = QFAC*QCOMP(QSPEC(IQ2,KQSP))/QINF
      CALL PLOT(XMOD(1.0-SSPEC(IQ1)),YMOD(YPLT1)-0.03,3)
      CALL PLOT(XMOD(1.0-SSPEC(IQ1)),YMOD(YPLT1)+0.03,2)
      CALL PLOT(XMOD(1.0-SSPEC(IQ2)),YMOD(YPLT2)-0.03,3)
      CALL PLOT(XMOD(1.0-SSPEC(IQ2)),YMOD(YPLT2)+0.03,2)
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      RETURN
      END


      SUBROUTINE QSPPLT(IQSPL1,IQSPL2,KQSP,NT)
C------------------------------------------
C     Plots KQSP-th Qspec(s) distribution
C     between indices IQSPL1..IQSPL2
C------------------------------------------
C
      INCLUDE 'XFOIL.INC'
      INCLUDE 'XDES.INC'
C
C---- statement function for compressible Karman-Tsien velocity
      QCOMP(G) = G*(1.0-TKLAM) / (1.0 - TKLAM*(G/QINF)**2)
C
C---- go over chosen intervals
      DO I=IQSPL1+1, IQSPL2
        DS = SSPEC(I) - SSPEC(I-1)
C
C------ plot Qpsec using NT sub-intervals for smooth curve
        IPL = 3
        DO IT=0, NT
          SSPT = SSPEC(I-1) + DS*FLOAT(IT)/FLOAT(NT)
          QSPT = SEVAL(SSPT,QSPEC(1,KQSP),QSPECP(1,KQSP),SSPEC,NSP)
          XPLT = 1.0 - SSPT
          YPLT = QFAC*QCOMP(QSPT)/QINF
          CALL PLOT(XMOD(XPLT),YMOD(YPLT),IPL)
          IPL = 2
        ENDDO
      ENDDO
C
      RETURN
      END


 
 
      SUBROUTINE IQSGET
C------------------------------------------------------------
C     Sets target segment endpoint indices from cursor input.
C------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      DIMENSION IQNEW(2)
      CHARACTER*1 KCHAR
      INCLUDE 'XDES.INC'
C
C---- statement function for compressible Karman-Tsien velocity
      QCOMP(G) = G*(1.0-TKLAM) / (1.0 - TKLAM*(G/QINF)**2)
C
      IF(.NOT.LQSPPL) THEN
       CALL QPLINI(.FALSE.)
       CALL QSPLOT
      ENDIF
C
      SH = 0.01*XSF
C
      CALL GETCOLOR(ICOL0)
C
      IQNEW(1) = 0
      IQNEW(2) = 0
      WRITE(*,*)
      WRITE(*,*) 'Mark off segment endpoints'
      WRITE(*,*)
      DO 10 IE=1, 2
C
C------ get cursor location from user
    5   CALL GETCURSORXY(XE,YE,KCHAR)
        DMIN = 1.0E9
        IQNEW(IE) = 1
        KQMIN = 1
C
C------ search all Qspec lines only for first selected point
        IF(IE.EQ.1) THEN
          KQSP1 = 1
          KQSPN = NQSP
        ELSE
          KQSP1 = KQTARG
          KQSPN = KQTARG
        ENDIF
C
C------ find plot point closest to cursor point
        DO 102 KQSP=KQSP1, KQSPN
          DO 1024 I=1, NSP
            GCOMP = QCOMP(QSPEC(I,KQSP))/QINF
            XPNT = XMOD(1.0-SSPEC(I))
            YPNT = YMOD(QFAC*GCOMP)
            DIST = (XE - XPNT)**2 + (YE - YPNT)**2
            IF(DIST.GT.DMIN) GO TO 1024
              DMIN = DIST
              IQNEW(IE) = I
              KQMIN = KQSP
 1024     CONTINUE
 102    CONTINUE
C
C------ nearest point to first clicked point sets target line
        IF(IE.EQ.1) KQTARG = KQMIN
C
        CALL NEWCOLORNAME('red')
        I = IQNEW(IE)
        QSCOMP = QCOMP(QSPEC(I,KQTARG))/QINF 
        CALL PLOT(XMOD(1.0-SSPEC(I)),YMOD(QFAC*QSCOMP)-0.03,3)
        CALL PLOT(XMOD(1.0-SSPEC(I)),YMOD(QFAC*QSCOMP)+0.03,2)
        CALL NEWCOLOR(ICOL0)
        CALL PLFLUSH
   10 CONTINUE
C
      IF(IQNEW(1).EQ.IQNEW(2)) THEN
       WRITE(*,*) '***  Endpoints must be distinct  ***'
       WRITE(*,*) '***  NEW SEGMENT NOT MARKED OFF  ***'
       RETURN
      ENDIF
C
      IQ1 = MIN0(IQNEW(1),IQNEW(2))
      IQ2 = MAX0(IQNEW(1),IQNEW(2))
C
      LIQSET = .TRUE.
      RETURN
      END




      SUBROUTINE SPLQSP(KQSP)
C------------------------------------------------------
C     Splines Qspec(s).  The end intervals are treated
C     specially to avoid Gibbs-type problems from 
C     blindly splining to the stagnation point.
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- usual spline with natural end BCs
      CALL SPLIND(QSPEC(2,KQSP),QSPECP(2,KQSP),SSPEC(2),NSP-2,
     &            -999.0,-999.0)
C
ccC---- pseudo-monotonic spline with simple secant slope calculation
cc      CALL SPLINA(QSPEC(2,KQSP),QSPECP(2,KQSP),SSPEC(2),NSP-2)
C
C---- end intervals are splined separately with natural BCs at
C     the trailing edge and matching slopes at the interior points
C
      I = 1
      CALL SPLIND(QSPEC(I,KQSP),QSPECP(I,KQSP),SSPEC(I),2,
     &            -999.0,QSPECP(I+1,KQSP))
C
      I = NSP-1
      CALL SPLIND(QSPEC(I,KQSP),QSPECP(I,KQSP),SSPEC(I),2,
     &            QSPECP(I,KQSP),-999.0)
C
      RETURN
      END


      SUBROUTINE SMOOQ(KQ1,KQ2,KQSP)
C--------------------------------------------
C     Smooths Qspec(s) inside target segment
C--------------------------------------------
      INCLUDE 'XFOIL.INC'
C
cC---- calculate smoothing coordinate
ccc      IF(NSP.EQ.NC1) THEN
cC
cC------ mapping inverse: use circle plane coordinate
c        I = 1
c        W8(I) = 0.0
c        DO 10 I=2, NSP
c          SINW = 2.0*SIN( 0.25*(WC(I)+WC(I-1)) )
c          SINWE = SINW**(1.0-AGTE)
cC
c          DSDW = SINWE * EXP( REAL(0.5*(PIQ(I)+PIQ(I-1)) ))
c          W8(I) = W8(I-1) + (WC(I)-WC(I-1))/DSDW
c   10   CONTINUE
c        DO 11 I=1, NSP
c          W8(I) = W8(I)/W8(NSP)
c 11     CONTINUE
cC
cC------ do not smooth first and last intervals in circle plane
c        KQ1 = MAX(IQ1,2)
c        KQ2 = MIN(IQ2,NSP-1)
cC
ccc      ELSE
C
C------ mixed inverse: use arc length coordinate
        DO 15 I=1, NSP
          W8(I) = SSPEC(I)
   15   CONTINUE
C
ccc      ENDIF
C
C
      IF(KQ2-KQ1 .LT. 2) THEN
       WRITE(*,*) 'Segment is too short.  No smoothing possible.'
       RETURN
      ENDIF
C
C---- set smoothing length ( ~ distance over which data is smeared )
      SMOOL = 0.002*(W8(NSP) - W8(1))
CCC   CALL ASKR('Enter Qspec smoothing length^',SMOOL)
C
C---- set up tri-diagonal system for smoothed Qspec
      SMOOSQ = SMOOL**2
      DO 20 I=KQ1+1, KQ2-1
        DSM = W8(I  ) - W8(I-1)
        DSP = W8(I+1) - W8(I  )
        DSO = 0.5*(W8(I+1) - W8(I-1))
C
        W1(I) =  SMOOSQ * (         - 1.0/DSM) / DSO
        W2(I) =  SMOOSQ * ( 1.0/DSP + 1.0/DSM) / DSO  +  1.0
        W3(I) =  SMOOSQ * (-1.0/DSP          ) / DSO
   20 CONTINUE
C
C---- set fixed-Qspec end conditions
      W2(KQ1) = 1.0
      W3(KQ1) = 0.0
C
      W1(KQ2) = 0.0
      W2(KQ2) = 1.0
C
      IF(LQSLOP) THEN
C----- also enforce slope matching at endpoints
       I = KQ1 + 1
       DSM = W8(I  ) - W8(I-1)
       DSP = W8(I+1) - W8(I  )
       DS  = W8(I+1) - W8(I-1)
       W1(I) = -1.0/DSM - (DSM/DS)/DSM
       W2(I) =  1.0/DSM + (DSM/DS)/DSM + (DSM/DS)/DSP
       W3(I) =                         - (DSM/DS)/DSP
       QSPP1 = W1(I)*QSPEC(I-1,KQSP)
     &       + W2(I)*QSPEC(I  ,KQSP)
     &       + W3(I)*QSPEC(I+1,KQSP)
C
       I = KQ2 - 1
       DSM = W8(I  ) - W8(I-1)
       DSP = W8(I+1) - W8(I  )
       DS  = W8(I+1) - W8(I-1)
       W1(I) =                           (DSP/DS)/DSM
       W2(I) = -1.0/DSP - (DSP/DS)/DSP - (DSP/DS)/DSM
       W3(I) =  1.0/DSP + (DSP/DS)/DSP
       QSPP2 = W1(I)*QSPEC(I-1,KQSP)
     &       + W2(I)*QSPEC(I  ,KQSP)
     &       + W3(I)*QSPEC(I+1,KQSP)
C
       QSPEC(KQ1+1,KQSP) = QSPP1
       QSPEC(KQ2-1,KQSP) = QSPP2
      ENDIF
C
C
C---- solve for smoothed Qspec array
      CALL TRISOL(W2(KQ1),W1(KQ1),W3(KQ1),QSPEC(KQ1,KQSP),(KQ2-KQ1+1))
C
C
cc      IF(LQSYM) THEN
cc        DO 40 I=KQ1+1, KQ2-1
cc          QSPEC(NSP-I+1,KQSP) = -QSPEC(I,KQSP)
cc 40     CONTINUE
cc      ENDIF
C
      RETURN
      END
 

      FUNCTION QINCOM(QC,QINF,TKLAM)
C-------------------------------------
C     Sets incompressible speed from
C     Karman-Tsien compressible speed
C-------------------------------------
C
      IF(TKLAM.LT.1.0E-4 .OR. ABS(QC).LT.1.0E-4) THEN
C----- for nearly incompressible case or very small speed, use asymptotic
C      expansion of singular quadratic formula to avoid numerical problems
       QINCOM = QC/(1.0 - TKLAM)
      ELSE
C----- use quadratic formula for typical case
       TMP = 0.5*(1.0 - TKLAM)*QINF/(QC*TKLAM)
       QINCOM = QINF*TMP*(SQRT(1.0 + 1.0/(TKLAM*TMP**2)) - 1.0)
      ENDIF
      RETURN
      END 

 
 
 
 
      SUBROUTINE GAMQSP(KQSP)
C------------------------------------------------
C     Sets Qspec(s,k) from current speed Q(s).
C------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      ALQSP(KQSP) = ALGAM
      CLQSP(KQSP) = CLGAM
      CMQSP(KQSP) = CMGAM
C
      DO 10 I=1, NSP
        QSPEC(I,KQSP) = QGAMM(I)
 10   CONTINUE
C
C---- zero out Qspec DOFs
      QDOF0 = 0.0
      QDOF1 = 0.0
      QDOF2 = 0.0
      QDOF3 = 0.0
C
      CALL SPLQSP(KQSP)
C
C---- reset target segment endpoints
      IF(.NOT.LIQSET) THEN
       IQ1 = 1
       IQ2 = NSP
      ENDIF
C
      RETURN
      END


      SUBROUTINE SYMQSP(KQSP)
C-----------------------------------------
C     Forces symmetry of Qspec(KQSP) array
C-----------------------------------------
      INCLUDE 'XFOIL.INC'
C
      ALQSP(KQSP) = 0.
      CLQSP(KQSP) = 0.
      CMQSP(KQSP) = 0.
C
      SSPMID = 0.5*(SSPEC(NSP) - SSPEC(1))
      DO 10 I=1, (NSP+1)/2
        SSPEC(I) = SSPMID + 0.5*(SSPEC(I)      - SSPEC(NSP-I+1)  )
        QSPEC(I,KQSP) =     0.5*(QSPEC(I,KQSP) - QSPEC(NSP-I+1,KQSP))
 10   CONTINUE
C
      DO 15 I=(NSP+1)/2+1, NSP
        SSPEC(I)      = -SSPEC(NSP-I+1)      + 2.0*SSPMID
        QSPEC(I,KQSP) = -QSPEC(NSP-I+1,KQSP)
 15   CONTINUE
C
C---- zero out Qspec DOFs
      QDOF0 = 0.0
      QDOF1 = 0.0
      QDOF2 = 0.0
      QDOF3 = 0.0
C
      CALL SPLQSP(KQSP)
C
      WRITE(*,1000) KQSP
 1000 FORMAT(/' Qspec',I2,'  made symmetric')
C
      RETURN
      END



      SUBROUTINE MIXED(KQSP,NITERQ)
C-------------------------------------------------
C     Performs a mixed-inverse calculation using 
C     the specified surface speed array QSPEC.
C-------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- distance of internal control point ahead of sharp TE
C-    (fraction of smaller panel length adjacent to TE)
      BWT = 0.1
C
      COSA = COS(ALFA)
      SINA = SIN(ALFA)
      CALL SCALC(X,Y,S,N)
C
C---- zero-out and set DOF shape functions
      DO 1 I=1, N
        QF0(I) = 0.0
        QF1(I) = 0.0
        QF2(I) = 0.0
        QF3(I) = 0.0
    1 CONTINUE
C
C---- set DOF shape functions and specified speed
      DO 2 I=IQ1, IQ2
        FS = (S(I)-S(IQ1)) / (S(IQ2)-S(IQ1))
CCC        QF0(I) = (1.0-FS)**2
CCC        QF1(I) = FS**2
        QF0(I) = 1.0 - FS
        QF1(I) = FS
        IF(LCPXX) THEN
         QF2(I) = EXP(-5.0*     FS )
         QF3(I) = EXP(-5.0*(1.0-FS))
        ELSE
         QF2(I) = 0.0
         QF3(I) = 0.0
        ENDIF
        GAM(I) = QSPEC(I,KQSP) + QDOF0*QF0(I) + QDOF1*QF1(I)
     &                         + QDOF2*QF2(I) + QDOF3*QF3(I)
    2 CONTINUE
C
   99 CONTINUE
C
C---- perform Newton iterations on the new geometry
      DO 1000 ITER=1, NITERQ
C
      DO 3 I=1, N+5
        DO 31 J=1, N+5
          Q(I,J) = 0.
   31   CONTINUE
    3 CONTINUE
C
C---- calculate normal direction vectors along which the nodes move
      CALL NCALC(X,Y,S,N,NX,NY)
C
C---- go over all nodes, setting up  Psi = Psi0  equations
      DO 20 I=1, N
        CALL PSILIN(I,X(I),Y(I),NX(I),NY(I),PSI,PSI_N,.TRUE.,.FALSE.)
C
        DZDN(I) = DZDN(I) + PSI_N
C
C------ fill columns for specified geometry location
        DO 201 J=1, IQ1-1
          Q(I,J) = Q(I,J) + DZDG(J)
  201   CONTINUE
C
C------ fill columns for specified surface speed location
        DO 202 J=IQ1, IQ2
          Q(I,J) = Q(I,J) + DZDN(J)
  202   CONTINUE
C
C------ fill columns for specified geometry location
        DO 203 J=IQ2+1, N
          Q(I,J) = Q(I,J) + DZDG(J)
  203   CONTINUE
C
C------ set residual
        DQ(I) = PSIO - PSI
C
C------ fill global unknown columns
        Q(I,N+1) = Q(I,N+1) - 1.0
        Q(I,N+2) = Q(I,N+2) + Z_QDOF0
        Q(I,N+3) = Q(I,N+3) + Z_QDOF1
        Q(I,N+4) = Q(I,N+4) + Z_QDOF2
        Q(I,N+5) = Q(I,N+5) + Z_QDOF3
   20 CONTINUE
C
C---- set up Kutta condition
      DQ(N+1) = -( GAM(1) + GAM(N) )
      CALL GAMLIN(N+1,1,1.0)
      CALL GAMLIN(N+1,N,1.0)
C
      IF(SHARP) THEN
C----- set zero internal velocity in TE corner 
C
C----- set TE bisector angle
       AG1 = ATAN2(-YP(1),-XP(1)    )
       AG2 = ATANC( YP(N), XP(N),AG1)
       ABIS = 0.5*(AG1+AG2)
       CBIS = COS(ABIS)
       SBIS = SIN(ABIS)
C
C----- minimum panel length adjacent to TE
       DS1 = SQRT( (X(1)-X(2)  )**2 + (Y(1)-Y(2)  )**2 )
       DS2 = SQRT( (X(N)-X(N-1))**2 + (Y(N)-Y(N-1))**2 )
       DSMIN = MIN( DS1 , DS2 )
C
C----- control point on bisector just ahead of TE point
       XBIS = XTE - BWT*DSMIN*CBIS
       YBIS = YTE - BWT*DSMIN*SBIS
ccc       write(*,*) xbis, ybis
C
C----- set velocity component along bisector line
       CALL PSILIN(0,XBIS,YBIS,-SBIS,CBIS,PSI,QBIS,.FALSE.,.TRUE.)
C
CCC--- RES = DQDGj*Gamj + DQDMj*Massj + QINF*(COSA*CBIS + SINA*SBIS)
       RES = QBIS
C
       DO J=1, N+5
         Q(N,J) = 0.
       ENDDO
C
C----- dRes/dgamj
       DO J=1, N
         CALL GAMLIN(N,J, DQDG(J) )
         Q(N,J) = DQDG(J)
       ENDDO
C
C----- dRes/dPsio
       Q(N,N+1) = 0.
C
C----- -dRes/dUinf
       DQ(N) = -RES
      ENDIF
C
C---- pinned IQ1 point condition
      Q(N+2,IQ1) = 1.0
      DQ(N+2) = 0.0
C
C---- pinned IQ2 point condition
      Q(N+3,IQ2) = 1.0
      DQ(N+3) = 0.0
C
      IF(IQ1.GT.1 .AND. LCPXX) THEN
C----- speed regularity IQ1 condition
       RES = GAM(IQ1-1)      - 2.0*  GAM(IQ1)      +   GAM(IQ1+1)
     &  - (QSPEC(IQ1-1,KQSP) - 2.0*QSPEC(IQ1,KQSP) + QSPEC(IQ1+1,KQSP) )
       CALL GAMLIN(N+4,IQ1-1, 1.0)
       CALL GAMLIN(N+4,IQ1  ,-2.0)
       CALL GAMLIN(N+4,IQ1+1, 1.0)
       DQ(N+4) = -RES
      ELSE
C----- zero DOF condition
       Q(N+4,N+4) = 1.0
       DQ(N+4) = -QDOF2
      ENDIF
C
      IF(IQ2.LT.N .AND. LCPXX) THEN
C----- speed regularity IQ2 condition
       RES = GAM(IQ2-1)      - 2.0*  GAM(IQ2)      +   GAM(IQ2+1)
     &  - (QSPEC(IQ2-1,KQSP) - 2.0*QSPEC(IQ2,KQSP) + QSPEC(IQ2+1,KQSP) )
       CALL GAMLIN(N+5,IQ2-1, 1.0)
       CALL GAMLIN(N+5,IQ2  ,-2.0)
       CALL GAMLIN(N+5,IQ2+1, 1.0)
       DQ(N+5) = -RES
      ELSE
C----- zero DOF condition
       Q(N+5,N+5) = 1.0
       DQ(N+5) = -QDOF3
      ENDIF
C
      CALL GAUSS(IQX,N+5,Q,DQ,1)
C
      INMAX = 0
      IGMAX = 0
      DNMAX = 0.0
      DGMAX = 0.0
C
C---- update surface speed GAM before target segment
      DO 100 I=1, IQ1-1
        GAM(I) = GAM(I) + DQ(I)
        IF(ABS(DQ(I)) .GT. ABS(DGMAX)) THEN
         DGMAX = DQ(I)
         IGMAX = I
        ENDIF
  100 CONTINUE
C
C---- update panel nodes inside target segment
      DO 110 I=IQ1, IQ2
        X(I) = X(I) + NX(I)*DQ(I)
        Y(I) = Y(I) + NY(I)*DQ(I)
        IF(ABS(DQ(I)) .GT. ABS(DNMAX)) THEN
         DNMAX = DQ(I)
         INMAX = I
        ENDIF
  110 CONTINUE
C
C---- update surface speed GAM after target segment
      DO 120 I=IQ2+1, N
        GAM(I) = GAM(I) + DQ(I)
        IF(ABS(DQ(I)) .GT. ABS(DGMAX)) THEN
         DGMAX = DQ(I)
         IGMAX = I
        ENDIF
  120 CONTINUE
C
C---- update gloabal variables
      PSIO  = PSIO  + DQ(N+1)
      QDOF0 = QDOF0 + DQ(N+2)
      QDOF1 = QDOF1 + DQ(N+3)
      QDOF2 = QDOF2 + DQ(N+4)
      QDOF3 = QDOF3 + DQ(N+5)
C
      COSA = COS(ALFA)
      SINA = SIN(ALFA)
      CALL SCALC(X,Y,S,N)
C
C---- set correct surface speed over target segment including DOF contributions
      DO 140 I=IQ1, IQ2
        GAM(I) = QSPEC(I,KQSP) + QDOF0*QF0(I) + QDOF1*QF1(I)
     &                         + QDOF2*QF2(I) + QDOF3*QF3(I)
  140 CONTINUE
C
C---- update everything else
      CALL TECALC
      CALL CLCALC(N,X,Y,GAM,GAM_A,ALFA,MINF,QINF, XCMREF,YCMREF,
     &            CL,CM,CDP, CL_ALF,CL_MSQ)
      WRITE(*,2000) DNMAX,INMAX,DGMAX,IGMAX,CL
     &             ,DQ(N+2),DQ(N+3)
     &             ,DQ(N+4),DQ(N+5)
 2000 FORMAT(/' dNmax =',E10.3,I4,'   dQmax =',E10.3,I4,'    CL =',F7.4
     &       /' dQf1  =',E10.3,4X,'   dQf2  =',E10.3
     &       /' dQf3  =',E10.3,4X,'   dQf4  =',E10.3)
C
      IF(ABS(DNMAX).LT.5.0E-5 .AND. ABS(DGMAX).LT.5.0E-4) THEN
       WRITE(*,*)
       WRITE(*,*) 'New current airfoil generated'
       WRITE(*,*) 'Old buffer  airfoil unchanged'
       RETURN
      ENDIF
C
 1000 CONTINUE
      WRITE(*,*) 'Not quite converged.  Can EXEC again if necessary.'
      RETURN
C
      END

 
      SUBROUTINE GAMLIN(I,J,COEF)
C-------------------------------------------------------------------
C     Adds on Jacobian entry for point I due to node speed GAM at J.
C     GAM is either a local unknown if outside target segment,
C     or dependent on global Qspec DOF's if inside target segment.
C-------------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      IF(J.GE.IQ1 .AND. J.LE.IQ2) THEN
C----- inside target segment
       Q(I,N+2) = Q(I,N+2) + COEF*QF0(J)
       Q(I,N+3) = Q(I,N+3) + COEF*QF1(J)
       Q(I,N+4) = Q(I,N+4) + COEF*QF2(J)
       Q(I,N+5) = Q(I,N+5) + COEF*QF3(J)
      ELSE
C----- outside target segment
       Q(I,J) = Q(I,J) + COEF
      ENDIF
      RETURN
      END
