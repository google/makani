

      SUBROUTINE BLPLOT
C------------------------------------------------------
C      Plots various BL variables in x from a menu.
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      INCLUDE 'BLPAR.INC'
      CHARACTER*4 COMAND
ccc   CHARACTER*4 CHDUM
      REAL XXBL(IVX,2), XXTR(2)
      REAL WS(IVX,2), XS(IVX,2), WS2(IVX,2)
      REAL HK(IVX,2), ANU(IVX,2)
      INTEGER NSIDE(2), IBL1(2), IBL2(2)
C
      CHARACTER*128 COMARG
      CHARACTER*80 FILDEF, LINE
      CHARACTER*32 COLNAM
      CHARACTER*2 FILSUF(12)
      CHARACTER*1 CXXBL
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
      DIMENSION DYGARR(10)
      INTEGER NUMBL(2)
C
      EXTERNAL PLCHAR, PLSLAN, PLMATH
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
      DATA FILSUF / 'hk', 'dT', 'dB', 'ue', 'cf',
     &              'cd', 'nc', 'ct', 'rt', 'rl',
     &              'G2', 'be' /
C
C---- number of grid intervals per axis annotation interval
      NGR = 2
C
C---- clear plot-type indicator (no plot yet)
      KPLOT = 0
C
C---- symbol size
      SH = 0.2*CH
C
C---- get current color for restoration
      CALL GETCOLOR(ICOL0)
C
C---- set up Cartesian BL x-arrays for plotting
      IF(IXBLP.EQ.1) THEN
       CALL XBLSET(IVX,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      ELSE
       CALL SBLSET(IVX,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      ENDIF
C
C
cC---- max BL coordinate plotted
c      XBLMAX = 1.6
cC
c      DO 3 IS=1, 2
c        DO 31 IBL=2, NSIDE(IS)
c          IF(XXBL(IBL,IS) .LT. XBLMAX) NUMBL(IS) = IBL-1
c 31     CONTINUE
c 3    CONTINUE
C
C---- plot width (standard width = 1)
      XWIDTH = 0.9
C
C---- 1 / total enthalpy
      HSTINV = GAMM1*(MINF/QINF)**2 / (1.0 + 0.5*GAMM1*MINF**2)
C
C---- Sutherland's const./To   (assumes stagnation conditions are at STP)
      HVRAT = 0.35
C
C============================
C---- set default plot axis limits
 5    CONTINUE
C
      CHX = 0.99*(XTE - XLE)
      XTEW = XTE + 0.4*CHX
      CALL SCALIT(1,CHX,0.0,XFAC)
      XDEL = 1.0/(5.0*XFAC)
      XMAX = AINT(ABS(XTEW)/XDEL + 0.05) * SIGN(XDEL,XTEW)
      XMIN = AINT(ABS(XLE )/XDEL + 0.05) * SIGN(XDEL,XLE )
C
      HKMAX = 6.0
      HKMIN = 0.0
      HKDEL = 1.0
C
      GAMAX = 5.0
      GAMIN = 0.0
      GADEL = 1.0
C
      BEMAX =  5.0
      BEMIN = -1.0
      BEDEL =  1.0
C
      CALL SCALIT(NUMBL(1),DSTR(2,1),0.0,YFAC)
      DSMAX = 1.0/YFAC
      DSMIN = 0.0
      DSDEL = (DSMAX-DSMIN)/5.0
C
      CALL SCALIT(NUMBL(2),DSTR(2,2),0.0,YFAC)
      DPMAX = 1.0/YFAC
      DPMIN = 0.0
      DPDEL = (DPMAX-DPMIN)/5.0
C
      UEMAX = 1.6
      UEMIN = 0.0
      UEDEL = 0.2
C
      TAUMAX = 0.0
      DISMAX = 0.0
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          TAUMAX = MAX(TAUMAX,TAU(IBL,IS))
          DISMAX = MAX(DISMAX,DIS(IBL,IS))
        ENDDO
      ENDDO
      QUE = 0.5*QINF**2
      CALL SCALIT(1,TAUMAX/QUE,0.0,YFAC)
      CFMAX = 0.5/YFAC
      CFMIN = 0.0
      CFDEL = (CFMAX-CFMIN)/5.0
C
      QRF = QINF
      CALL SCALIT(1,DISMAX/QRF**3,0.0,YFAC)
      DIMAX = 0.5/YFAC
      DIMIN = 0.0
      DIDEL = (DIMAX-DIMIN)/5.0
C
C
      ACR1 = MAX(1.0,ACRIT(1)+1.5,ACRIT(2)+1.5)
      CALL SCALIT(1,ACR1,0.0,YFAC)
      ANDEL = 1.0/(5.0*YFAC)
      ANMAX = ANDEL*AINT(ACR1/ANDEL + 0.6)
      ANMIN = 0.
C
      CMAX = 0.0
      DO IS=1, 2
        DO IBL=ITRAN(IS), NSIDE(IS)
          CMAX = MAX( CMAX , ABS(CTAU(IBL,IS)) )
        ENDDO
      ENDDO
      CALL SCALIT(1,CMAX,0.0,YFAC)
      CTMAX = 1.0/YFAC
      CTMIN = 0.0
      CTDEL = (CTMAX-CTMIN)/5.0
C
      RMAX = 0.0
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          RTHETA = REINF * UEDG(IBL,IS)*THET(IBL,IS)
          RMAX = MAX(RMAX,RTHETA)
        ENDDO
      ENDDO
      CALL SCALIT(1,RMAX,0.0,YFAC)
      RTMAX = 1.0/YFAC
      RTMIN = 0.0
      RTDEL = (RTMAX-RTMIN)/5.0
C
      RLMAX = 5.0
      RLMIN = 1.0
      RLDEL = 1.0
C
C
  500 CONTINUE
      CALL ASKC('..VPLO^',COMAND,COMARG)
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
      IF(COMAND.EQ.'?   ') GO TO 9
      IF(COMAND.EQ.'H   '.OR.
     &   COMAND.EQ.'HK  ') GO TO 10
      IF(COMAND.EQ.'DS  '.OR.
     &   COMAND.EQ.'DT  ') GO TO 20
      IF(COMAND.EQ.'DP  '.OR.
     &   COMAND.EQ.'DB  ') GO TO 30
      IF(COMAND.EQ.'UE  ') GO TO 40
      IF(COMAND.EQ.'CF  ') GO TO 50
      IF(COMAND.EQ.'CD  ') GO TO 60
      IF(COMAND.EQ.'N   ') GO TO 70
      IF(COMAND.EQ.'CT  ') GO TO 80
      IF(COMAND.EQ.'RT  ') GO TO 90
      IF(COMAND.EQ.'RTL ') GO TO 100
      IF(COMAND.EQ.'G   ') GO TO 110
      IF(COMAND.EQ.'BE  ') GO TO 120
      IF(COMAND.EQ.'DUMP') GO TO 140
      IF(COMAND.EQ.'OVER') GO TO 140
      IF(COMAND.EQ.'XPLO') GO TO 145
      IF(COMAND.EQ.'XLIM' .OR. COMAND.EQ.'X   ') GO TO 147
      IF(COMAND.EQ.'YLIM' .OR. COMAND.EQ.'Y   ') GO TO 148
      IF(COMAND.EQ.'BLOW' .OR. COMAND.EQ.'B   ') GO TO 150
      IF(COMAND.EQ.'RESE' .OR. COMAND.EQ.'R   ') GO TO 5
      IF(COMAND.EQ.'GRID') GO TO 152
      IF(COMAND.EQ.'SYMB') GO TO 153
      IF(COMAND.EQ.'LABE') GO TO 154
      IF(COMAND.EQ.'CLIP') GO TO 155
      IF(COMAND.EQ.'FRPL') GO TO 157
      IF(COMAND.EQ.'HARD') GO TO 160
      IF(COMAND.EQ.'SIZE') GO TO 165
      IF(COMAND.EQ.'ANNO') GO TO 170
      IF(COMAND.EQ.'Z   ') then
        call usetzoom(.true.,.true.)
        call replot(idev)
        go to 500
      endif
      IF(COMAND.EQ.'U   ') then
        call clrzoom
        call replot(idev)
        go to 500
      endif
C
      WRITE(*,1010) COMAND
      GO TO 500
C
 9    WRITE(*,1050)
      GO TO 500
C
C===================================================
C---- plot Hk
C
 10   KPLOT = 1
C
C---- fill kinematic shape parameter array
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          THI = THET(IBL,IS)
          DSI = DSTR(IBL,IS)
          UEI = UEDG(IBL,IS)
          UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
          AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
          CALL HKIN( DSI/THI, AMSQ, WS(IBL,IS), DUMMY, DUMMY)
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = HKMIN
      YMAX = HKMAX
      YDEL = HKDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
      CALL PLCHAR(-4.0*CH,YL-0.5*CH,1.4*CH,'H',0.0,1)
      CALL PLSUBS(-4.0*CH,YL-0.5*CH,1.4*CH,'k',0.0,1,PLCHAR)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
C----- plot equilibrium and actual  1/Ue dUe/dx
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXQ(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXD(2,1),XMIN,XSF,YMIN,YSF,7)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXQ(2,2),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXD(2,2),XMIN,XSF,YMIN,YSF,7)
C
cC---- plot 1.6/(1+Us)
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),USLP(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),USLP(2,2),XMIN,XSF,YMIN,YSF,4)
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
C---- plot top delta*, theta
   20 KPLOT = 2
      IS = 1
      YMIN = DSMIN
      YMAX = DSMAX
      YDEL = DSDEL
      GO TO 35
C
C===================================================
C---- plot bottom delta*, theta
   30 KPLOT = 3
      IS = 2
      YMIN = DPMIN
      YMAX = DPMAX
      YDEL = DPDEL
      GO TO 35
C
C
   35 CONTINUE
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
C
      CALL NEWPEN(3)
      IF((YMAX-YMIN)/YDEL .GT. 2.99) THEN
       YL1 = YSF*(YMAX-YMIN-0.5*YDEL)
       YL2 = YSF*(YMAX-YMIN-1.5*YDEL)
       YL3 = YSF*(YMAX-YMIN-2.5*YDEL)
      ELSE
       YL1 = YSF*(YMAX-YMIN-0.25*YDEL)
       YL2 = YSF*(YMAX-YMIN-0.50*YDEL)
       YL3 = YSF*(YMAX-YMIN-0.75*YDEL)
      ENDIF
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL NEWCOLOR(ICOLS(IS))
C
      IF(IS.EQ.1)
     & CALL PLCHAR(-4.5*CH,YL1-0.6*CH,1.3*CH,'Top',0.0, 3)
      IF(IS.EQ.2)
     & CALL PLCHAR(-4.5*CH,YL1-0.6*CH,1.3*CH,'Bot',0.0, 3)
      CALL PLMATH(-4.0*CH,YL2-0.6*CH,1.5*CH,'d'  ,0.0,1)
      CALL PLSUPS(-4.0*CH,YL2-0.6*CH,1.5*CH,'*'  ,0.0,1,PLCHAR)
      CALL PLMATH(-3.5*CH,YL3-0.6*CH,1.5*CH,'q'  ,0.0,1)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL XYLINE(NUMBL(IS),XXBL(2,IS),DSTR(2,IS),XMIN,XSF,YMIN,YSF,1)
      CALL XYLINE(NUMBL(IS),XXBL(2,IS),THET(2,IS),XMIN,XSF,YMIN,YSF,1)
      CALL XYLINE(NUMBL(IS),XXBL(2,IS),DELT(2,IS),XMIN,XSF,YMIN,YSF,2)
C
      IF(LBLSYM) THEN
       CALL XYSYMB(NUMBL(IS),XXBL(2,IS),DSTR(2,IS),
     &             XMIN,XSF,YMIN,YSF,SH,1)
       CALL XYSYMB(NUMBL(IS),XXBL(2,IS),THET(2,IS),
     &             XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C
C===================================================
C---- plot Ue
C
   40 KPLOT = 4
C
C---- fill compressible Ue arrays
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          UEI = UEDG(IBL,IS)
          WS(IBL,IS) = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = UEMIN
      YMAX = UEMAX
      YDEL = UEDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.0*CH,CXXBL,0.0,1)
      CALL PLSUBS(-5.0*CH,YL-0.5*CH,1.0*CH,'e'   ,0.0,1,PLCHAR)
      CALL PLCHAR(-5.0*CH,YL-0.5*CH,1.0*CH,'U /V',0.0,4)
      CALL PLMATH(999.0  ,999.0    ,1.0*CH,   '&',0.0,1)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C============================================
C---- plot Cf
C
   50 KPLOT = 5
C
      QUE = 0.5*QINF**2
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          WS(IBL,IS) = TAU(IBL,IS) / QUE
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = CFMIN
      YMAX = CFMAX
      YDEL = CFDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
      CALL PLCHAR(-3.5*CH,YL-0.6*CH,1.4*CH,'C',0.0,1)
      CALL PLSUBS(-3.5*CH,YL-0.6*CH,1.4*CH,'f',0.0,1,PLCHAR)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C============================================
C---- plot CD
C
   60 KPLOT = 6
C
      QRF = QINF
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          WS(IBL,IS) = DIS(IBL,IS) / QRF**3
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = DIMIN
      YMAX = DIMAX
      YDEL = DIDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL ,0.0,1)
      CALL PLCHAR(-3.5*CH,YL-0.6*CH,1.4*CH,'C' ,0.0,1)
      CALL PLMATH(-3.7*CH,YL-0.6*CH,1.5*CH,' `',0.0,2)
      CALL PLSUBS(-3.5*CH,YL-0.6*CH,1.4*CH,'D' ,0.0,1,PLSLAN)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C
C===================================================
C---- plot A/Ao
Cs
   70 KPLOT = 7
C
      IF(LFREQP) THEN
C----- fill Hk and nu arrays
       DO IS=1, 2
         DO IBL=2, NSIDE(IS)
           THI = THET(IBL,IS)
           DSI = DSTR(IBL,IS)
           UEI = UEDG(IBL,IS)
           UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
           AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
           CALL HKIN( DSI/THI, AMSQ, HK(IBL,IS), DUMMY, DUMMY)
C
           HERAT = (1.0 - 0.5*HSTINV*UEI **2)
     &           / (1.0 - 0.5*HSTINV*QINF**2)
           RHRAT = HERAT ** (1.0/GAMM1)
           ANU(IBL,IS) = SQRT(HERAT**3) * (1.0+HVRAT)/(HERAT+HVRAT)
     &                / (RHRAT * REINF)
         ENDDO
       ENDDO
      ENDIF
C
C---- set offsets and scalings
      YMIN = ANMIN
      YMAX = ANMAX
      YDEL = ANDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
C
      IF((YMAX-YMIN)/YDEL .GT. 1.99) THEN
       YL1 = YSF*(YMAX-YMIN-0.5*YDEL)
       YL2 = YSF*(YMAX-YMIN-1.5*YDEL)
      ELSE
       YL1 = YSF*(YMAX-YMIN-0.33*YDEL)
       YL2 = YSF*(YMAX-YMIN-0.67*YDEL)
      ENDIF
      CALL PLCHAR(-4.0*CH,YL1-0.6*CH,1.2*CH,'ln' ,0.0,2)
      CALL PLCHAR(-5.0*CH,YL2-0.6*CH,1.2*CH,'A/A',0.0,3)
      CALL PLSUBS(-2.6*CH,YL2-0.6*CH,1.2*CH,  '0',0.0,1,PLCHAR)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      DO IS=1, 2
        IF(LFREQP) THEN
         CALL NEWPEN(5)
        ELSE
         CALL NEWPEN(3)
        ENDIF
C
        CALL NEWCOLOR(ICOLS(IS))
        NBLS = ITRAN(IS) - 2
        CALL XYLINE(NBLS,XXBL(2,IS),CTAU(2,IS),XMIN,XSF,YMIN,YSF,1)
C
        IF(LBLSYM)
     &  CALL XYSYMB(NBLS,XXBL(2,IS),CTAU(2,IS),XMIN,XSF,YMIN,YSF,SH,1)
C
        IF(.NOT.TFORCE(IS)) THEN
         IBL = ITRAN(IS) - 1
         CALL PLOT((XXBL(IBL,IS)-XMIN)*XSF,(CTAU(IBL,IS)-YMIN)*YSF,3)
         CALL PLOT((XXTR(IS)    -XMIN)*XSF,(ACRIT(IS)   -YMIN)*YSF,2)
        ENDIF
C
        IF(LFREQP) THEN
C------- plot amplitudes of individual frequencies
         FREF = 1.0
         CHF = 0.6*CH
C
         CALL NEWPEN(1)
         IO = 2
         NBLS = ITRAN(IS) - 2
C
         CALL GETCOLORRGB(ICOLS(IS),IRED,IGRN,IBLU,COLNAM)
         CALL NEWCOLORRGB((IRED*2)/3,(IGRN*2)/3,(IBLU*2)/3)
         CALL FRPLOT(NBLS,XSSI(IO,IS),XXBL(IO,IS),
     &               HK(IO,IS),THET(IO,IS),UEDG(IO,IS),ANU(IO,IS),
     &               XXTR(IS), FREF,
     &               XMIN,XSF, YMIN,YSF, CHF)
        ENDIF
        IF(ACRIT(1) .NE. ACRIT(2)) THEN
         CALL DASH(0.0,XSF*(XMAX-XMIN),YSF*(ACRIT(IS)-YMIN))
        ENDIF
      ENDDO
C
      CALL NEWCOLOR(ICOL0)

      IF(ACRIT(1) .EQ. ACRIT(2)) THEN
        CALL DASH(0.0,XSF*(XMAX-XMIN),YSF*(ACRIT(1)-YMIN))
      ENDIF
C
      IF(LFREQP) THEN
C----- add label to plot
       XLAB = XSF*(MAX(XXBL(ITRAN(1),1),XXBL(ITRAN(2),2))-XMIN)
     &      + 9.0*CHF
       YLAB = 0.5*YSF*(YMAX-YMIN) + 0.5*CH
       CALL NEWPEN(2)
       CALL PLMATH(XLAB,YLAB,CH,'w   &',0.0,5)
       CALL PLCHAR(XLAB,YLAB,CH,' L/V ',0.0,5)
      ENDIF
C
C
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
C---- plot Ctau
C
   80 KPLOT = 8
C
C---- set offsets and scalings
      YMIN = CTMIN
      YMAX = CTMAX
      YDEL = CTDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
C
      IF((YMAX-YMIN)/YDEL .GT. 1.99) THEN
       YL1 = YSF*(YMAX-YMIN-0.5*YDEL)
       YL2 = YSF*(YMAX-YMIN-1.5*YDEL)
      ELSE
       YL1 = YSF*(YMAX-YMIN-0.33*YDEL)
       YL2 = YSF*(YMAX-YMIN-0.67*YDEL)
      ENDIF
C
      CALL PLMATH(-3.7*CH,YL1-0.6*CH,1.4*CH,' H',0.0,2)
      CALL PLCHAR(-3.7*CH,YL1-0.6*CH,1.4*CH,'C ',0.0,2)
      CALL PLSUBS(-3.7*CH,YL1-0.6*CH,1.4*CH,'t' ,0.0,1,PLMATH)
C
      CALL PLMATH(-3.7*CH,YL2-0.6*CH,1.4*CH,' H',0.0,2)
      CALL PLCHAR(-3.7*CH,YL2-0.6*CH,1.4*CH,'C ',0.0,2)
      CALL PLSUBS(-3.7*CH,YL2-0.6*CH,1.4*CH,'t' ,0.0,1,PLMATH)
      CALL PLCHAR(-1.8*CH,YL2-1.4*CH,0.7*CH,'eq',0.0,2)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      NBLS = NUMBL(1) - ITRAN(1) + 2
      NBLP = NUMBL(2) - ITRAN(2) + 2
      IT1 = ITRAN(1)
      IT2 = ITRAN(2)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NBLS,XXBL(IT1,1),CTAU(IT1,1),XMIN,XSF,YMIN,YSF,1)
cc    CALL XYLINE(NBLS,XXBL(IT1,1), CTQ(IT1,1),XMIN,XSF,YMIN,YSF,4)
      CALL XYLINE(NUMBL(1),XXBL(2,1),CTQ(2,1),XMIN,XSF,YMIN,YSF,4)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NBLP,XXBL(IT2,2),CTAU(IT2,2),XMIN,XSF,YMIN,YSF,1)
CCC   CALL XYLINE(NBLP,XXBL(IT2,2), CTQ(IT2,2),XMIN,XSF,YMIN,YSF,4)
cc      CALL XYLINE(IBLTE(2)-IT2+1,
cc     &                 XXBL(IT2,2), CTQ(IT2,2),XMIN,XSF,YMIN,YSF,4)
      CALL XYLINE(NUMBL(2),XXBL(2,2),CTQ(2,2),XMIN,XSF,YMIN,YSF,4)
C
      IF(LBLSYM) THEN
       CALL NEWCOLOR(ICOLS(1))
       CALL XYSYMB(NBLS,XXBL(IT1,1),CTAU(IT1,1),XMIN,XSF,YMIN,YSF,SH,1)
       CALL NEWCOLOR(ICOLS(2))
       CALL XYSYMB(NBLP,XXBL(IT2,2),CTAU(IT2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C
C===================================================
C---- plot Rtheta
C
 90   KPLOT = 9
C
C---- fill Rtheta arrays
      DO 801 IS=1, 2
        DO 8012 IBL=2, NSIDE(IS)
          UEI = UEDG(IBL,IS)
          UE  = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
          HERAT = (1.0 - 0.5*HSTINV*UEI **2)
     &          / (1.0 - 0.5*HSTINV*QINF**2)
          RHOE = HERAT ** (1.0/GAMM1)
          AMUE = SQRT(HERAT**3) * (1.0+HVRAT)/(HERAT+HVRAT)
          RTHETA = REINF * RHOE*UE*THET(IBL,IS)/AMUE
          WS(IBL,IS) = RTHETA
 8012   CONTINUE
 801  CONTINUE
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = RTMIN
      YMAX = RTMAX
      YDEL = RTDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-1)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG,NYG,DYG, LMASK2)
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
C
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(-4.4*CH,YL-0.6*CH,1.4*CH,'Re',0.0,2)
      CALL PLSUBS(-3.0*CH,YL-0.8*CH,1.4*CH, 'q',0.0,1,PLMATH)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C

cC---- fill and plot Rcrit arrays from AGS bypass transition model
c      DO 803 IS=1, 2
c        DO 8032 IBL=2, NSIDE(IS)
c          THI = THET(IBL,IS)
c          DSI = DSTR(IBL,IS)
c          UEI = UEDG(IBL,IS)
c          UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
c          AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
c          CALL HKIN( DSI/THI, AMSQ, HKI, DUMMY, DUMMY)
c          
c          TRB = 100.0 * EXP( -(ACRIT(IS)+8.43)/2.4 )
c          HMI = 1.0/(HKI-1.0)
c          GFUN = 3.625*LOG(TANH(10.0*(HMI - 0.55)) + 6.0)
c          RCR  = 163.0 + EXP((1.0-TRB/6.91)*GFUN)
cC
c          THH = TANH(10.0/(HKI-1.0) - 5.5)
c          RCR = 163.0 + 74.3*(0.55*THH + 1.0)*(0.94*ACRIT(IS) + 1.0)
cC
c          WS(IBL,IS) = RCR
c 8032   CONTINUE
c 803  CONTINUE
cC
c      CALL NEWPEN(2)
c      NUM1 = ITRAN(1) - 2
c      NUM2 = ITRAN(2) - 2
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUM1,XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,2)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUM2,XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,2)

      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
C---- plot log(Rtheta)
C
 100  KPLOT = 10
C
C---- fill log(Rtheta) arrays
      DO 901 IS=1, 2
        DO 9012 IBL=2, NSIDE(IS)
          UEI = UEDG(IBL,IS)
          UE  = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
          HERAT = (1.0 - 0.5*HSTINV*UE  **2)
     &          / (1.0 - 0.5*HSTINV*QINF**2)
          RHOE = HERAT ** (1.0/GAMM1)
          AMUE = SQRT(HERAT**3) * (1.0+HVRAT)/(HERAT+HVRAT)
          RTHETA = REINF * RHOE*UE*THET(IBL,IS)/AMUE
          WS(IBL,IS) = 0.
          IF(RTHETA.GT.0.0) WS(IBL,IS) = LOG10(RTHETA)
 9012   CONTINUE
 901  CONTINUE
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = RLMIN
      YMAX = RLMAX
      YDEL = RLDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG =       INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL
       CALL NEWPEN(1)
       KK = 10
       DO K=1, KK
         FRAC = FLOAT(K+1)/FLOAT(K)
         DYGARR(K) = DYG * LOG10(FRAC)
       ENDDO
       DO IG=1, NYG
         YG0 = DYG*FLOAT(IG-1)
         CALL PLGRID(0.0,YG0, NXG,DXG, KK-1+1000,DYGARR, LMASK2)
       ENDDO
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
C
      IF((YMAX-YMIN)/YDEL .GT. 1.99) THEN
       YL1 = YSF*(YMAX-YMIN-0.5*YDEL)
       YL2 = YSF*(YMAX-YMIN-1.5*YDEL)
      ELSE
       YL1 = YSF*(YMAX-YMIN-0.33*YDEL)
       YL2 = YSF*(YMAX-YMIN-0.67*YDEL)
      ENDIF
      CALL PLCHAR(-5.5*CH,YL1-0.6*CH,1.1*CH,'log' ,0.0,3)
      CALL PLSUBS(-3.3*CH,YL1-0.8*CH,1.1*CH,  '10',0.0,2,PLCHAR)
      CALL PLCHAR(-4.4*CH,YL2-0.6*CH,1.4*CH,'Re' ,0.0,2)
      CALL PLSUBS(-3.0*CH,YL2-0.8*CH,1.4*CH, 'q' ,0.0,1,PLMATH)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C

cC---- fill and plot Rcrit arrays from AGS bypass transition model
c      DO 903 IS=1, 2
c        DO 9032 IBL=2, NSIDE(IS)
c          THI = THET(IBL,IS)
c          DSI = DSTR(IBL,IS)
c          UEI = UEDG(IBL,IS)
c          UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
c          AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
c          CALL HKIN( DSI/THI, AMSQ, HKI, DUMMY, DUMMY)
cC          
c          TRB = 100.0 * EXP( -(ACRIT(IS)+8.43)/2.4 )
c          HMI = 1.0/(HKI-1.0)
c          GFUN = 3.625*LOG(TANH(10.0*(HMI - 0.55)) + 6.0)
c          RCR  = 163.0 + EXP((1.0-TRB/6.91)*GFUN)
cC
c          WS(IBL,IS) = LOG10(RCR)
c 9032   CONTINUE
c 903  CONTINUE
cC
c      CALL NEWPEN(2)
c      NUM1 = ITRAN(1) - 2
c      NUM2 = ITRAN(2) - 2
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUM1,XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,2)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUM2,XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,2)

      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
C---- plot G  (Clauser shape parameter)
C
 110  KPLOT = 11
C
C---- fill G array
      DO IS=1, 2
        DO IBL=2, NSIDE(IS)
          THI = THET(IBL,IS)
          DSI = DSTR(IBL,IS)
          UEI = UEDG(IBL,IS)
          UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
          AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
          CALL HKIN( DSI/THI, AMSQ, HKI, DUMMY, DUMMY)
          QLOC = 0.5*UC*UC / (1.0 + 0.5*GAMM1*AMSQ)**(1.0/GAMM1)
          CF = TAU(IBL,IS) / QLOC
          CFLIM = MAX( CF , 0.0001 )
          WS(IBL,IS) = ((HKI-1.0)/(GACON*HKI))**2 / (0.5*CFLIM)
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = GAMIN
      YMAX = GAMAX
      YDEL = GADEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
      CALL PLCHAR(-7.5*CH,YL-0.5*CH,1.4*CH,'G /A ',0.0,5)
      CALL PLMATH(-7.5*CH,YL-0.5*CH,1.4*CH,' 2  2',0.0,5)
ccc   CALL PLSUBS(-7.5*CH,YL-0.5*CH,1.4*CH,'k',0.0,1,PLCHAR)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
C----- plot equilibrium and actual  1/Ue dUe/dx
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXQ(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXD(2,1),XMIN,XSF,YMIN,YSF,7)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXQ(2,2),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXD(2,2),XMIN,XSF,YMIN,YSF,7)
C
cC---- plot 1.6/(1+Us)
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),USLP(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),USLP(2,2),XMIN,XSF,YMIN,YSF,4)
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
C---- plot beta  (Clauser pressure gradient parameter)
C
 120  KPLOT = 12
C
      DO IS=1, 2
        DO IBL=2, NBL(IS)
          DSO = DSTR(IBL,IS)
          UEO = UEDG(IBL,IS)
          IF    (IBL.EQ.IBLTE(IS) .OR. IBL.EQ.NSIDE(IS)) THEN
           UEM = UEDG(IBL-1,IS)
           UEP = UEDG(IBL  ,IS)
           XIM = XSSI(IBL-1,IS)
           XIP = XSSI(IBL  ,IS)
          ELSEIF(IBL.EQ.IBLTE(IS)+1) THEN
           UEM = UEDG(IBL  ,IS)
           UEP = UEDG(IBL+1,IS)
           XIM = XSSI(IBL  ,IS)
           XIP = XSSI(IBL+1,IS)
          ELSE
           UEM = UEDG(IBL-1,IS)
           UEP = UEDG(IBL+1,IS)
           XIM = XSSI(IBL-1,IS)
           XIP = XSSI(IBL+1,IS)
          ENDIF
          UCO = UEO * (1.0-TKLAM) / (1.0 - TKLAM*(UEO/QINF)**2)
          UCM = UEM * (1.0-TKLAM) / (1.0 - TKLAM*(UEM/QINF)**2)
          UCP = UEP * (1.0-TKLAM) / (1.0 - TKLAM*(UEP/QINF)**2)
C
          DUDS = (UCP-UCM) / (XIP-XIM)
          AMSQ = UCO*UCO*HSTINV / (GAMM1*(1.0 - 0.5*UCO*UCO*HSTINV))
          QLOC = 0.5*UCO*UCO / (1.0 + 0.5*GAMM1*AMSQ)**(1.0/GAMM1)
          CF = TAU(IBL,IS) / QLOC
          CFLIM = MAX( CF , 0.0001 )
          WS(IBL,IS) = -DSO*DUDS / (UCO * 0.5*CFLIM)
        ENDDO
      ENDDO
C
      CALL PLTINI
      CALL PLOT(8.0*CH,6.0*CH,-3)
C
C---- set offsets and scalings
      YMIN = BEMIN
      YMAX = BEMAX
      YDEL = BEDEL
C
      XSF = XWIDTH/(XMAX-XMIN)
      YSF = PLOTAR/(YMAX-YMIN)
C
C---- draw and annotate axes
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,XSF*(XMAX-XMIN),XSF*XDEL,XMIN,XDEL,CH,-2)
      CALL YAXIS(0.0,0.0,YSF*(YMAX-YMIN),YSF*YDEL,YMIN,YDEL,CH,-2)
C
      IF(LBLGRD) THEN
       NXG = NGR * INT((XMAX-XMIN)/XDEL + 0.001)
       NYG = NGR * INT((YMAX-YMIN)/YDEL + 0.001)
       DXG = XSF*XDEL / FLOAT(NGR)
       DYG = YSF*YDEL / FLOAT(NGR)
       CALL NEWPEN(1)
       CALL PLGRID(0.0,0.0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      CALL NEWPEN(3)
      XL = XSF*(XMAX-XMIN-1.5*XDEL)
      YL = YSF*(YMAX-YMIN-1.5*YDEL)
      CALL PLCHAR(XL-0.6*CH,-3.5*CH,1.2*CH,CXXBL,0.0,1)
      CALL PLMATH(-2.0*CH,YL-0.5*CH,1.4*CH,'b',0.0,1)
ccc   CALL PLSUBS(-2.0*CH,YL-0.5*CH,1.4*CH,'k',0.0,1,PLCHAR)
C
      IF(LVLAB) CALL VLABEL(0.0,YSF*(YMAX-YMIN),CH,
     &                      NAME,
     &                      REINF,MINF,ACRIT(1),ALFA,
     &                      CL,CD,XOCTR(1),XOCTR(2),
     &                      ICOLS(1),ICOLS(2),LVCONV)
C
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
C
C---- plot upper and lower distributions
      CALL NEWPEN(3)
      CALL NEWCOLOR(ICOLS(1))
      CALL XYLINE(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,1)
      CALL NEWCOLOR(ICOLS(2))
      CALL XYLINE(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,1)
C
      IF(LBLSYM) THEN
        CALL NEWCOLOR(ICOLS(1))
        CALL XYSYMB(NUMBL(1),XXBL(2,1),WS(2,1),XMIN,XSF,YMIN,YSF,SH,1)
        CALL NEWCOLOR(ICOLS(2))
        CALL XYSYMB(NUMBL(2),XXBL(2,2),WS(2,2),XMIN,XSF,YMIN,YSF,SH,1)
      ENDIF
C
C----- plot equilibrium and actual  1/Ue dUe/dx
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXQ(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(1),XXBL(2,1),GUXD(2,1),XMIN,XSF,YMIN,YSF,7)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXQ(2,2),XMIN,XSF,YMIN,YSF,4)
c      CALL XYLINE(NUMBL(2),XXBL(2,2),GUXD(2,2),XMIN,XSF,YMIN,YSF,7)
C
cC---- plot 1.6/(1+Us)
c      CALL NEWCOLOR(ICOLS(1))
c      CALL XYLINE(NUMBL(1),XXBL(2,1),USLP(2,1),XMIN,XSF,YMIN,YSF,4)
c      CALL NEWCOLOR(ICOLS(2))
c      CALL XYLINE(NUMBL(2),XXBL(2,2),USLP(2,2),XMIN,XSF,YMIN,YSF,4)
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
ccc      CALL ASKC('Hit <cr>^',CHDUM,COMARG)
      GO TO 500
C
C===================================================
 140  IF(KPLOT.EQ.0) THEN
       WRITE(*,*) 'No current plot'
       GO TO 500
      ENDIF
C
      IF(COMARG(1:1).NE.' ') THEN
       FNAME = COMARG
      ELSE
C----- no argument... get it somehow
       IF(NPREFIX.GT.0) THEN
C------ offer default using existing prefix
        FILDEF = PREFIX(1:NPREFIX) // '.' // FILSUF(KPLOT)
        WRITE(*,1220) FILDEF
 1220   FORMAT(/' Enter filename:  ', A)
        READ(*,1000) FNAME
        CALL STRIP(FNAME,NFN)
        IF(NFN.EQ.0) FNAME = FILDEF
       ELSE
C------ nothing available... just ask for filename
        CALL ASKS('Enter filename^',FNAME)
       ENDIF
      ENDIF
C
      IF(COMAND.EQ.'DUMP') GO TO 122
      IF(COMAND.EQ.'OVER') GO TO 124
C
C--------------------------------------------
 122  CONTINUE
      LU = 19
      OPEN(LU,FILE=FNAME,STATUS='UNKNOWN')
      REWIND(LU)
C
      WRITE(LU,1001) '#  ', NAME
      WRITE(LU,1003) '#  alpha =', ALFA/DTOR
      WRITE(LU,1003) '#  Mach  =', MINF
      WRITE(LU,1002) '#  Reyn  =', INT(REINF+0.5)
      IF(ACRIT(1) .EQ. ACRIT(2)) THEN
       WRITE(LU,1003) '#  Ncrit =', ACRIT(1)
      ELSE
       WRITE(LU,1003) '#  Ncrit =', ACRIT(1), ACRIT(2)
      ENDIF
      WRITE(LU,1001) '#'
      WRITE(LU,1001)
     & '#    x           ', FILSUF(KPLOT)
C         0.234510      0.234510
C
      DO IS = 1, 2
        IBL1(IS) = 2
        IBL2(IS) = NSIDE(IS)
      ENDDO
C
      DO IS = 1, 2
        DO IBL = 2, NSIDE(IS)
          IF(KPLOT.EQ.1) THEN
           THI = THET(IBL,IS)
           DSI = DSTR(IBL,IS)
           UEI = UEDG(IBL,IS)
           UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
           AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
           CALL HKIN( DSI/THI, AMSQ, WS(IBL,IS), DUMMY, DUMMY)
           XS(IBL,IS) = XXBL(IBL,IS)

ccc           WS(IBL,IS) = TSTR(IBL,IS) / THET(IBL,IS)
ccc           XS(IBL,IS) = XSSI(IBL,IS)   !%%%
C
          ELSEIF(KPLOT.EQ.2 .AND. IS.EQ.1) THEN
           IBL1(1) = 2
           IBL1(2) = 2
           IBL2(1) = NSIDE(IS)
           IBL2(2) = NSIDE(IS)
           WS(IBL,1) = DSTR(IBL,IS)
           WS(IBL,2) = THET(IBL,IS)
           XS(IBL,1) = XXBL(IBL,IS)
           XS(IBL,2) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.3 .AND. IS.EQ.2) THEN
           IBL1(1) = 2
           IBL1(2) = 2
           IBL2(1) = NSIDE(IS)
           IBL2(2) = NSIDE(IS)
           WS(IBL,1) = DSTR(IBL,IS)
           WS(IBL,2) = THET(IBL,IS)
           XS(IBL,1) = XXBL(IBL,IS)
           XS(IBL,2) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.4) THEN
           UEI = UEDG(IBL,IS)
           WS(IBL,IS) = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.5) THEN
           WS(IBL,IS) = TAU(IBL,IS) / QUE
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.6) THEN
           QRF = QINF
           WS(IBL,IS) = DIS(IBL,IS) / QRF**3
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.7) THEN
           IBL1(IS) = 2
           IBL2(IS) = ITRAN(IS) - 1
           WS(IBL,IS) = CTAU(IBL,IS)
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.8) THEN
           IBL1(IS) = ITRAN(IS)
           IBL2(IS) = NSIDE(IS)
           WS(IBL,IS) = CTAU(IBL,IS)
           XS(IBL,IS) = XXBL(IBL,IS)
           WS2(IBL,IS) = CTQ(IBL,IS)
C
          ELSEIF(KPLOT.EQ.9 .OR. KPLOT.EQ.10) THEN
C--------- 1 / (total enthalpy)
           HSTINV = GAMM1*(MINF/QINF)**2 / (1.0 + 0.5*GAMM1*MINF**2)
C
C--------- fill Rtheta arrays
           UEI = UEDG(IBL,IS)
           UE  = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
           HERAT = (1.0 - 0.5*HSTINV*UE  **2)
     &           / (1.0 - 0.5*HSTINV*QINF**2)
           RHOE = HERAT ** (1.0/GAMM1)
           AMUE = SQRT(HERAT**3) * (1.0+HVRAT)/(HERAT+HVRAT)
           RTHETA = REINF * RHOE*UE*THET(IBL,IS)/AMUE
C
           IF(KPLOT.EQ.9) THEN
            WS(IBL,IS) = RTHETA
           ELSE
            WS(IBL,IS) = LOG10( MAX(RTHETA,1.0) )
           ENDIF
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.11) THEN
C--------- G
           THI = THET(IBL,IS)
           DSI = DSTR(IBL,IS)
           UEI = UEDG(IBL,IS)
           UC = UEI * (1.0-TKLAM) / (1.0 - TKLAM*(UEI/QINF)**2)
           AMSQ = UC*UC*HSTINV / (GAMM1*(1.0 - 0.5*UC*UC*HSTINV))
           CALL HKIN( DSI/THI, AMSQ, HKI, DUMMY, DUMMY)
           QLOC = 0.5*UC*UC / (1.0 + 0.5*GAMM1*AMSQ)**(1.0/GAMM1)
           CF = TAU(IBL,IS) / QLOC
           CFLIM = MAX( CF , 0.0001 )
           WS(IBL,IS) = ((HKI-1.0)/(GACON*HKI))**2 / (0.5*CFLIM)
           XS(IBL,IS) = XXBL(IBL,IS)
C
          ELSEIF(KPLOT.EQ.12) THEN
C--------- beta
           DSO = DSTR(IBL,IS)
           UEO = UEDG(IBL,IS)
           IF    (IBL.EQ.IBLTE(IS) .OR. IBL.EQ.NSIDE(IS)) THEN
            UEM = UEDG(IBL-1,IS)
            UEP = UEDG(IBL  ,IS)
            XIM = XSSI(IBL-1,IS)
            XIP = XSSI(IBL  ,IS)
           ELSEIF(IBL.EQ.IBLTE(IS)+1) THEN
            UEM = UEDG(IBL  ,IS)
            UEP = UEDG(IBL+1,IS)
            XIM = XSSI(IBL  ,IS)
            XIP = XSSI(IBL+1,IS)
           ELSE
            UEM = UEDG(IBL-1,IS)
            UEP = UEDG(IBL+1,IS)
            XIM = XSSI(IBL-1,IS)
            XIP = XSSI(IBL+1,IS)
           ENDIF
           UCO = UEO * (1.0-TKLAM) / (1.0 - TKLAM*(UEO/QINF)**2)
           UCM = UEM * (1.0-TKLAM) / (1.0 - TKLAM*(UEM/QINF)**2)
           UCP = UEP * (1.0-TKLAM) / (1.0 - TKLAM*(UEP/QINF)**2)
C
           DUDS = (UCP-UCM) / (XIP-XIM)
           AMSQ = UCO*UCO*HSTINV / (GAMM1*(1.0 - 0.5*UCO*UCO*HSTINV))
           QLOC = 0.5*UCO*UCO / (1.0 + 0.5*GAMM1*AMSQ)**(1.0/GAMM1)
           CF = TAU(IBL,IS) / QLOC
           CFLIM = MAX( CF , 0.0001 )
           WS(IBL,IS) = -DSO*DUDS / (UCO * 0.5*CFLIM)
           XS(IBL,IS) = XXBL(IBL,IS)
          ENDIF
        ENDDO
      ENDDO
C
      DO IS = 1, 2
        DO IBL = IBL1(IS), IBL2(IS)
          IF(KPLOT.EQ.8) THEN
           WRITE(LU,8500) XS(IBL,IS), WS(IBL,IS), WS2(IBL,IS)
          ELSE
           WRITE(LU,8500) XS(IBL,IS), WS(IBL,IS)
          ENDIF
 8500     FORMAT(1X,3G14.6)
        ENDDO
        WRITE(LU,1000)
      ENDDO
C
      CLOSE(LU)
      GO TO 500
C
C--------------------------------------------
 124  CONTINUE
      LU = 19
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=128)
C
      IS = 1
      IBL = 1
C
      IBL1(IS) = 2
      IBL2(IS) = 2
C
C---- read and echo header lines
 125  READ(LU,1000,END=127) LINE
      IF(LINE(1:1).EQ.'#') THEN
       WRITE(*,*) LINE(2:80)
       GO TO 125
      ENDIF
C....................................
C---- begin data reading loop
 126  CONTINUE
      IF(LINE(1:10).EQ.'          ') THEN
       IF(IS.EQ.2) THEN
C------ empty line... go plot data
        GO TO 127
       ELSE
C------ blank line denotes start of new side
        IS = IS + 1
        IBL = 1
        IBL1(IS) = 2
        IBL2(IS) = 2
        READ(LU,1000,END=127) LINE
        GO TO 126
       ENDIF
      ENDIF
C
      IF(IBL.GE.IVX) GO TO 127
C
C---- read data from line string
      IBL = IBL+1
      READ(LINE,*,ERR=129) XS(IBL,IS), WS(IBL,IS)
      IBL2(IS) = IBL
C
      READ(LU,1000,END=127) LINE
      GO TO 126
C....................................
C
 127  CLOSE(LU)
C
C---- plot data
      CALL GETCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      IF(LCLIP) CALL NEWCLIP(MAX(XCLIP1,0.),MIN(XCLIP2,XSF*(XMAX-XMIN)),
     &                       MAX(YCLIP1,0.),MIN(YCLIP2,YSF*(YMAX-YMIN)))
      DO IS = 1, 2
        IF(KPLOT.EQ.2) THEN
         CALL NEWCOLOR(ICOLS(1))
        ELSEIF(KPLOT.EQ.3) THEN
         CALL NEWCOLOR(ICOLS(2))
        ELSE
         CALL NEWCOLOR(ICOLS(IS))
        ENDIF
C
        IBL = IBL1(IS)
        NNBL = IBL2(IS) - IBL1(IS) + 1
cc      SSH = 1.3*SH
        SSH = 0.7*CH
        CALL XYSYMB(NNBL,XS(IBL,IS),WS(IBL,IS),
     &              XMIN,XSF,YMIN,YSF,SSH,5)
      ENDDO
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIP(XCLIP1,XCLIP2,YCLIP1,YCLIP2)
      CALL PLFLUSH
C
C---- set new default prefix
      KDOT = INDEX(FNAME,'.')
      IF(KDOT.EQ.0) THEN
       PREFIX = FNAME
      ELSE
       PREFIX = FNAME(1:KDOT-1)
      ENDIF
      CALL STRIP(PREFIX,NPREFIX)
      GO TO 500
C
 128  CONTINUE
      WRITE(*,*) 'File OPEN error'
      GO TO 500
C
 129  CONTINUE
      WRITE(*,*) 'File READ error'
      CLOSE(LU)
      GO TO 500
C
C===================================================
 145  CONTINUE
      IF(IXBLP.EQ.1) THEN
       IXBLP = 2
       CALL SBLSET(IVX,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      ELSE
       IXBLP = 1
       CALL XBLSET(IVX,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      ENDIF
C
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 147  CONTINUE
      IF(NINPUT.GE.3) THEN
       XMIN = RINPUT(1)
       XMAX = RINPUT(2)
       XDEL = RINPUT(3)
      ELSE
       WRITE(*,9101) XMIN, XMAX, XDEL
 9101  FORMAT(/' Currently, Xmin,Xmax,Xdel =', 3F11.4,
     &        /' Enter new  Xmin,Xmax,Xdel: ', $ )
       READ(*,*,ERR=147) XMIN, XMAX, XDEL
      ENDIF
C
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 148  IF(KPLOT.EQ.0) THEN
       WRITE(*,*) 'No current plot'
       GO TO 500
      ENDIF
C
      IF(NINPUT.GE.3) THEN
       YMIN = RINPUT(1)
       YMAX = RINPUT(2)
       YDEL = RINPUT(3)
      ELSE
       WRITE(*,9201) YMIN, YMAX, YDEL
 9201  FORMAT(/' Currently, Ymin,Ymax,Ydel =', 3F11.4,
     &        /' Enter new  Ymin,Ymax,Ydel :    ', $ )
       READ(*,*,ERR=140) YMIN, YMAX, YDEL
      ENDIF
C
      IF     (KPLOT.EQ.1) THEN
       HKMIN = YMIN
       HKMAX = YMAX
       HKDEL = YDEL
      ELSE IF(KPLOT.EQ.2) THEN
       DSMIN = YMIN
       DSMAX = YMAX
       DSDEL = YDEL
      ELSE IF(KPLOT.EQ.3) THEN
       DPMIN = YMIN
       DPMAX = YMAX
       DPDEL = YDEL
      ELSE IF(KPLOT.EQ.4) THEN
       UEMIN = YMIN
       UEMAX = YMAX
       UEDEL = YDEL
      ELSE IF(KPLOT.EQ.5) THEN
       CFMIN = YMIN
       CFMAX = YMAX
       CFDEL = YDEL
      ELSE IF(KPLOT.EQ.6) THEN
       DIMIN = YMIN
       DIMAX = YMAX
       DIDEL = YDEL
      ELSE IF(KPLOT.EQ.7) THEN
       ANMIN = YMIN
       ANMAX = YMAX
       ANDEL = YDEL
      ELSE IF(KPLOT.EQ.8) THEN
       CTMIN = YMIN
       CTMAX = YMAX
       CTDEL = YDEL
      ELSE IF(KPLOT.EQ.9) THEN
       RTMIN = YMIN
       RTMAX = YMAX
       RTDEL = YDEL
      ELSE IF(KPLOT.EQ.10) THEN
       RLMIN = YMIN
       RLMAX = YMAX
CCC    RLDEL = YDEL
      ELSE IF(KPLOT.EQ.11) THEN
       GAMIN = YMIN
       GAMAX = YMAX
       GADEL = YDEL
      ELSE IF(KPLOT.EQ.12) THEN
       BEMIN = YMIN
       BEMAX = YMAX
       BEDEL = YDEL
      ENDIF
C
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,110) KPLOT+1
      GO TO 500
C
C===================================================
 150  IF(KPLOT.EQ.0) THEN
       WRITE(*,*) 'No current plot'
       GO TO 500
      ENDIF
C
      CALL OFFGET(XMIN,YMIN,XSF,YSF,XWIDTH,PLOTAR,.FALSE.,.TRUE.)
      XMAX = XWIDTH/XSF + XMIN
      YMAX = PLOTAR/YSF + YMIN
C
      CALL SCALIT(1,XMAX,XMIN,XFAC)
      XDEL = 1.0 / (5.0*XFAC)
      SGNMIN = SIGN(1.0,XMIN)
      SGNMAX = SIGN(1.0,XMAX)
      XMIN = XDEL * AINT(ABS(XMIN/XDEL) - 0.5)*SGNMIN
      XMAX = XDEL * AINT(ABS(XMAX/XDEL) + 0.5)*SGNMAX
C
      CALL SCALIT(1,YMAX,YMIN,YFAC)
      YDEL = 1.0 / (5.0*YFAC)
      SGNMIN = SIGN(1.0,YMIN)
      SGNMAX = SIGN(1.0,YMAX)
      YMIN = YDEL * AINT(ABS(YMIN/YDEL) - 0.5)*SGNMIN
      YMAX = YDEL * AINT(ABS(YMAX/YDEL) + 0.5)*SGNMAX
C
      IF     (KPLOT.EQ.1) THEN
       HKMIN = YMIN
       HKMAX = YMAX
       HKDEL = YDEL
      ELSE IF(KPLOT.EQ.2) THEN
       DSMIN = YMIN
       DSMAX = YMAX
       DSDEL = YDEL
      ELSE IF(KPLOT.EQ.3) THEN
       DPMIN = YMIN
       DPMAX = YMAX
       DPDEL = YDEL
      ELSE IF(KPLOT.EQ.4) THEN
       UEMIN = YMIN
       UEMAX = YMAX
       UEDEL = YDEL
      ELSE IF(KPLOT.EQ.5) THEN
       CFMIN = YMIN
       CFMAX = YMAX
       CFDEL = YDEL
      ELSE IF(KPLOT.EQ.6) THEN
       DIMIN = YMIN
       DIMAX = YMAX
       DIDEL = YDEL
      ELSE IF(KPLOT.EQ.7) THEN
       ANMIN = YMIN
       ANMAX = YMAX
       ANDEL = YDEL
      ELSE IF(KPLOT.EQ.8) THEN
       CTMIN = YMIN
       CTMAX = YMAX
       CTDEL = YDEL
      ELSE IF(KPLOT.EQ.9) THEN
       RTMIN = YMIN
       RTMAX = YMAX
       RTDEL = YDEL
      ELSE IF(KPLOT.EQ.10) THEN
       RLMIN = YMIN
       RLMAX = YMAX
CCC    RLDEL = YDEL
      ELSE IF(KPLOT.EQ.11) THEN
       GAMIN = YMIN
       GAMAX = YMAX
       GADEL = YDEL
      ELSE IF(KPLOT.EQ.12) THEN
       BEMIN = YMIN
       BEMAX = YMAX
       BEDEL = YDEL
      ENDIF
C
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 152  LBLGRD = .NOT.LBLGRD
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 153  LBLSYM = .NOT.LBLSYM
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 154  LVLAB = .NOT.LVLAB
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 155  LCLIP = .NOT.LCLIP
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 157  LFREQP = .NOT.LFREQP
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C
C===================================================
 160  IF(LPLOT) CALL PLEND
      LPLOT = .FALSE.
      CALL REPLOT(IDEVRP)
      GO TO 500
C
C===================================================
 165  IF(NINPUT.GE.1) THEN
       SIZE = RINPUT(1)
      ELSE
       WRITE(*,*) 'Current plot-object size =', SIZE
       CALL ASKR('Enter new plot-object size^',SIZE)
      ENDIF
C
      GO TO (500,10,20,30,40,50,60,70,80,90,100,110,120) KPLOT+1
      GO TO 500
C===================================================
 170  IF(LPLOT) THEN
       CALL ANNOT(CH)
      ELSE
       WRITE(*,*) 'No active plot to annotate'
      ENDIF
      GO TO 500
C...................................................................
 1000 FORMAT(A)
 1001 FORMAT(A,A,A,A)
 1002 FORMAT(A,I9)
 1003 FORMAT(A,2F9.4)
 1010 FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
 1050 FORMAT(/'   <cr>   Return to OPER menu'
     &      //'   H      Plot kinematic shape parameter'
     &       /'   DT     Plot top    side Dstar and Theta'
     &       /'   DB     Plot bottom side Dstar and Theta'
     &       /'   UE     Plot edge velocity'
     &       /'   CF     Plot skin friction coefficient'
     &       /'   CD     Plot dissipation coefficient'
     &       /'   N      Plot amplification ratio'
     &       /'   CT     Plot max shear coefficient'
     &       /'   RT     Plot Re_theta'
     &       /'   RTL    Plot log(Re_theta)'
     &      //'   DUMP f Write current plot variable to file'
     &       /'   OVER f Overlay current plot variable from file'
     &      //'   X  rrr Change x-axis limits'
     &       /'   Y  rrr Change y-axis limits on current plot'
     &       /'   XPLO   Toggle plotting  vs  x or s'
     &      //'   BLOW   Cursor blowup of current plot'
     &       /'   RESE   Reset to default x,y-axis limits'
     &       /'   SIZE r Change absolute plot-object size'
     &       /'  .ANNO   Annotate plot'
     &       /'   HARD   Hardcopy current plot'
     &      //'   GRID   Toggle grid plotting'
     &       /'   SYMB   Toggle node-symbol plotting'
     &       /'   LABE   Toggle label plotting'
     &       /'   CLIP   Toggle line-plot clipping'
     &       /'   FRPL   Toggle TS frequency plotting')
      END ! BLPLOT



      SUBROUTINE XBLSET(IDIM,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      INCLUDE 'XFOIL.INC'
      REAL XXBL(IDIM,2), XXTR(2)
      INTEGER NSIDE(2), NUMBL(2)
      CHARACTER*(*) CXXBL
C
      DO IS=1, 2
        DO IBL=2, NBL(IS)
          I = IPAN(IBL,IS)
          XXBL(IBL,IS) = X(I)
          XXTR(IS) = XLE + (XTE-XLE)*XOCTR(IS) - (YTE-YLE)*YOCTR(IS)
        ENDDO
      ENDDO
C
      NSIDE(1) = NBL(2) + IBLTE(1) - IBLTE(2)
      NSIDE(2) = NBL(2)
C
      DO IBLW=1, NBL(2)-IBLTE(2)
        XXBL(IBLTE(1)+IBLW,1) = XXBL(IBLTE(2)+IBLW,2)
      ENDDO
C
      NUMBL(1) = NSIDE(1) - 1
      NUMBL(2) = NSIDE(2) - 1
C
      CXXBL = 'X'
C
      RETURN
      END ! XBLSET



      SUBROUTINE SBLSET(IDIM,XXBL,XXTR,NSIDE,NUMBL,CXXBL)
      INCLUDE 'XFOIL.INC'
      REAL XXBL(IDIM,2), XXTR(2)
      INTEGER NSIDE(2), NUMBL(2)
      CHARACTER*(*) CXXBL
C
      DO IS=1, 2
        DO IBL=2, NBL(IS)
          XXBL(IBL,IS) = XSSI(IBL,IS)
          XXTR(IS) = XSSITR(IS)
        ENDDO
      ENDDO
C
      NSIDE(1) = NBL(2) + IBLTE(1) - IBLTE(2)
      NSIDE(2) = NBL(2)
C
      DO IBLW=1, NBL(2)-IBLTE(2)
         XXBL(IBLTE(1)+IBLW,1) = XXBL(IBLTE(2)+IBLW,2)
     & + XXBL(IBLTE(1)     ,1) - XXBL(IBLTE(2)     ,2)
      ENDDO
C
      NUMBL(1) = NSIDE(1) - 1
      NUMBL(2) = NSIDE(2) - 1
C
      CXXBL = 'S'
C
      RETURN
      END ! SBLSET



      SUBROUTINE VLABEL(X0,Y0,CH,
     &                  NAME,
     &                  REINF,MINF,ACRIT,ALFA,
     &                  CL,CD,XTRT,XTRB,ICOL1,ICOL2,LVCONV)
      CHARACTER*(*) NAME
      REAL MINF
      LOGICAL LVCONV
C
      EXTERNAL PLCHAR
C
      ADEG = ALFA * 45.0/ATAN(1.0)
      CHN = 1.2*CH
C
      CALL GETCOLOR(ICOL0)
C
      X1 = X0
      X2 = X0 + 16.0*CH
      X3 = X0 + 30.0*CH
      X4 = X0 + 45.0*CH
C
      Y1 = Y0 + 1.5*CH
      Y2 = Y0 + 4.0*CH
      Y3 = Y0 + 6.8*CH
C
      CALL NEWPEN(3)
      CALL PLCHAR(X1,Y3,CHN,NAME,0.0,-1)
C
C
      CALL NEWPEN(2)
      CALL PLCHAR(X1       ,Y2,CH,'Ma = ',0.0,5)
      CALL PLNUMB(X1+5.0*CH,Y2,CH, MINF  ,0.0,4)
C
      CALL PLCHAR(X1        ,Y1        ,CH,'Re = '   ,0.0,5)
      NDIG = 3
      IF(REINF .GE. 9.9995E6) NDIG = 2
      IF(REINF .GE. 99.995E6) NDIG = 1
      IF(REINF .GE. 999.95E6) NDIG = 0
      CALL PLNUMB(X1+ 5.0*CH,Y1        ,CH, REINF*1.E-6,0.0,NDIG)
      CALL PLMATH(X1+10.1*CH,Y1+0.10*CH,0.80*CH,'#'  ,0.0,1)
      CALL PLCHAR(X1+10.9*CH,Y1        ,     CH,'10' ,0.0,2)
      CALL PLMATH(X1+12.9*CH,Y1        ,1.10*CH,  '6',0.0,1)
C
C
      CALL PLMATH(X2       ,Y2,1.2*CH,'a',0.0,1)
      CALL PLCHAR(X2       ,Y2,CH,'   = ',0.0,5)
      CALL PLNUMB(X2+5.0*CH,Y2,CH, ADEG  ,0.0,4)
      CALL PLMATH(999.0    ,Y2,CH,'"'    ,0.0,1)
C
      CALL PLCHAR(X2       ,Y1,CH,'N  = ',0.0,5)
      CALL PLSUBS(X2       ,Y1,CH,'cr'   ,0.0,2,PLCHAR)
      CALL PLNUMB(X2+5.0*CH,Y1,CH,ACRIT  ,0.0,2)
C
C
      CALL PLCHAR(X3       ,Y2,CH,'C  = ',0.0,5)
      CALL PLSUBS(X3       ,Y2,CH, 'L'   ,0.0,1,PLCHAR)
      CALL PLNUMB(X3+5.0*CH,Y2,CH, CL    ,0.0,4)
C
      CALL PLCHAR(X3       ,Y1,CH,'C  = ',0.0,5)
      CALL PLSUBS(X3       ,Y1,CH, 'D'   ,0.0,1,PLCHAR)
      CALL PLNUMB(X3+5.0*CH,Y1,CH, CD    ,0.0,5)
C
C
      CALL NEWCOLOR(ICOL1)
      CALL PLCHAR(X4       ,Y2,CH,'T:x /c = ',0.0,9)
      CALL PLSUBS(X4+2.0*CH,Y2,0.85*CH,'tr'   ,0.0,2,PLCHAR)
      CALL PLNUMB(X4+9.0*CH,Y2,CH, XTRT      ,0.0,4)
C
      CALL NEWCOLOR(ICOL2)
      CALL PLCHAR(X4       ,Y1,CH,'B:x /c = ',0.0,9)
      CALL PLSUBS(X4+2.0*CH,Y1,0.85*CH,'tr'   ,0.0,2,PLCHAR)
      CALL PLNUMB(X4+9.0*CH,Y1,CH, XTRB      ,0.0,4)
C
C
      IF(.NOT.LVCONV) THEN
       CALL NEWCOLORNAME('red')
       XL = X1 + CHN*FLOAT(LEN(NAME)+1)
       CALL PLCHAR(XL,Y3,CHN,'* NOT CONVERGED *',0.0,17)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
      RETURN
      END ! VLABEL

