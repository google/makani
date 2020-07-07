C***********************************************************************
C    Module:  xplots.f
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

      SUBROUTINE PLTINI
      INCLUDE 'XFOIL.INC'
C
C---- terminate old plot if any
      IF(LPLOT) CALL PLEND
C
C---- initialize new plot
      IF(LLAND) THEN
        SIGNFR =  SCRNFR
      ELSE
        SIGNFR = -SCRNFR
      ENDIF
      CALL PLOPEN(SIGNFR,IPSLU,IDEV)
      LPLOT = .TRUE.
C
C---- set X-window size in inches (might have been resized by user)
      CALL GETWINSIZE(XWIND,YWIND)
C
C---- draw plot page outline offset by margins
      CALL NEWPEN(5)
      IF(XMARG .GT. 0.0) THEN
        CALL PLOTABS(      XMARG,      YMARG,3)
        CALL PLOTABS(      XMARG,YPAGE-YMARG,2)
        CALL PLOTABS(XPAGE-XMARG,      YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
      ENDIF
      IF(YMARG .GT. 0.0) THEN
        CALL PLOTABS(      XMARG,      YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,      YMARG,2)
        CALL PLOTABS(      XMARG,YPAGE-YMARG,3)
        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
      ENDIF
      CALL NEWPEN(1)
C
      CALL PLOTABS(XMARG,YMARG,-3)
      CALL NEWCLIPABS( XMARG, XPAGE-XMARG, YMARG, YPAGE-YMARG )
C
      CALL NEWFACTOR(SIZE)
C
      RETURN
      END ! PLTINI


 
      SUBROUTINE PANPLT
C-----------------------------------------------------
C     Shows panel nodes on current airfoil geometry.
C-----------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      XPLT(XX) = (XX - XOFP)*GSF
      YPLT(YY) = (YY - YOFP)*GSF
C
C---- length of normal tick mark showing panel node
      DSN = 0.01*CHORD
C
      XMIN = X(1)
      XMAX = X(1)
      YMIN = Y(1)
      YMAX = Y(1)
      DO 10 I=2, N
        XMIN = MIN(X(I),XMIN)
        XMAX = MAX(X(I),XMAX)
        YMIN = MIN(Y(I),YMIN)
        YMAX = MAX(Y(I),YMAX)
 10   CONTINUE
C
C---- set scale, offsets, to center airfoil in plot area
      XRANGE = MAX(1.0E-9, XMAX-XMIN)
      YRANGE = MAX(1.0E-9, YMAX-YMIN)
      GSF  = MIN( 1.0/XRANGE , PLOTAR/YRANGE )
      XOFP = XMIN - 0.5*(1.0   -GSF*XRANGE)/GSF - 0.05/GSF
      YOFP = YMIN - 0.5*(PLOTAR-GSF*YRANGE)/GSF - 0.05/GSF
C
      CALL PLTINI
C
      CALL GETCOLOR(ICOL0)
C
C---- plot axial chord line
      CALL NEWCOLORNAME('green')
      CALL NEWPEN(1)
      CALL PLOT(XPLT(XLE),YPLT(YLE),3)
      CALL PLOT(XPLT(XTE),YPLT(YTE),2)
C
C---- add tick marks
      DO 20 IT=1, 9
        XOC = FLOAT(IT)/10.0
        XT = XLE + XOC*(XTE-XLE)
        YT = YLE + XOC*(YTE-YLE)
C
        DT = 0.003
        IF(IT.EQ.5) DT = 0.005
        DTX = -DT*(YTE-YLE)
        DTY =  DT*(XTE-XLE)
C
        CALL PLOT(XPLT(XT+DTX),YPLT(YT+DTY),3)
        CALL PLOT(XPLT(XT-DTX),YPLT(YT-DTY),2)
   20 CONTINUE
C
      CALL NEWCOLOR(ICOL0)
C
      I = 1
      CALL PLOT(XPLT(X(I)),YPLT(Y(I)),3)
C
      XOCM = (  (X(I)-XLE)*(XTE-XLE)
     &        + (Y(I)-YLE)*(YTE-YLE) ) / CHORD**2
      DO 40 I=1, N
        XOCI = (  (X(I)-XLE)*(XTE-XLE)
     &          + (Y(I)-YLE)*(YTE-YLE) ) / CHORD**2
C
        IF(I.GT.1) THEN
         IF(S(I).GT.SLE .AND. S(I-1).LE.SLE) THEN
          XOCM = 0.0
          CALL NEWCOLOR(ICOL0)
         ENDIF
        ENDIF
C
        IF(S(I).LT.SLE) THEN
C-------- upper surface
          IF(XOCI.LT.XSREF2 .AND. XOCM.GT.XSREF2) THEN
            FRAC = (XSREF2-XOCM)/(XOCI-XOCM)
            XF = X(I-1) + FRAC*(X(I)-X(I-1))
            YF = Y(I-1) + FRAC*(Y(I)-Y(I-1))
            CALL PLOT(XPLT(XF),YPLT(YF),2)
            CALL NEWCOLORNAME('magenta')
          ENDIF
          IF(XOCI.LT.XSREF1 .AND. XOCM.GT.XSREF1) THEN
            FRAC = (XSREF1-XOCM)/(XOCI-XOCM)
            XF = X(I-1) + FRAC*(X(I)-X(I-1))
            YF = Y(I-1) + FRAC*(Y(I)-Y(I-1))
            CALL PLOT(XPLT(XF),YPLT(YF),2)
            CALL NEWCOLOR(ICOL0)
          ENDIF
        ELSE
C-------- lower surface
          IF(XOCI.GT.XPREF1 .AND. XOCM.LT.XPREF1) THEN
            FRAC = (XPREF1-XOCM)/(XOCI-XOCM)
            XF = X(I-1) + FRAC*(X(I)-X(I-1))
            YF = Y(I-1) + FRAC*(Y(I)-Y(I-1))
            CALL PLOT(XPLT(XF),YPLT(YF),2)
            CALL NEWCOLORNAME('magenta')
          ENDIF
          IF(XOCI.GT.XPREF2 .AND. XOCM.LT.XPREF2) THEN
            FRAC = (XPREF2-XOCM)/(XOCI-XOCM)
            XF = X(I-1) + FRAC*(X(I)-X(I-1))
            YF = Y(I-1) + FRAC*(Y(I)-Y(I-1))
            CALL PLOT(XPLT(XF),YPLT(YF),2)
            CALL NEWCOLOR(ICOL0)
          ENDIF
        ENDIF
C
        CALL PLOT(XPLT(X(I)),YPLT(Y(I)),2)
        CALL PLOT(XPLT(X(I)+DSN*NX(I)),YPLT(Y(I)+DSN*NY(I)),2)
        CALL PLOT(XPLT(X(I)),YPLT(Y(I)),3)
        XOCM = XOCI
   40 CONTINUE
C
      CALL CANG(X,Y,N,0, IMAX,AMAX)
      CH2 = 0.9*CH
C
      CALL PLOTABS(XMARG,YPAGE-YMARG,3)
      CALL GETLASTXY(XPL,YPL)
      XPL = XPL + 2.0*CH
      YPL = YPL - 3.0*CH
C
      CALL PLCHAR(XPL,YPL,CH,'Current airfoil paneling',0.0,-1)
C
      YPL = YPL - 2.4*CH
      CALL PLCHAR(XPL,YPL,CH2,'No. panel nodes: ',0.0,-1)
      RNUM = FLOAT(N) + 0.1
      CALL PLNUMB(999.0,YPL,CH2,RNUM  ,0.0,-1)
C
      YPL = YPL - 2.4*CH
      CALL PLCHAR(XPL,YPL,CH2,'Max panel angle: ',0.0,-1)
      CALL PLNUMB(999.0,YPL,CH2,AMAX,0.0,2)
      CALL PLMATH(999.0,YPL,CH2,'"' ,0.0,1)
C
      CALL PLFLUSH

      CALL PLEND
      LPLOT = .FALSE.
C
      RETURN
      END ! PANPLT
 

      SUBROUTINE CPX
C-----------------------------------------
C     Plots Cp vs x, integrated forces, 
C     parameters, and reference data.
C-----------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- set x location of label
      XPLT = 0.70
      IF(LFOREF) XPLT = 0.52
C
C---- size and type of reference-data symbol
      SH = 0.7*CH
      ISYM = 5
C
C---- Cp scaling factor
      PFAC = PLOTAR/(CPMAX-CPMIN)
C
C---- determine airfoil box size and location
      CALL AIRLIM(N,X,Y,XMIN,XMAX,YMIN,YMAX)
C
C---- y-offset for airfoil in  Cp vs x  plot
      FACA = FACAIR/(XMAX-XMIN)
      XOFA = XOFAIR*(XMAX-XMIN) - XMIN
      YOFA = YOFAIR*(XMAX-XMIN) - YMAX - CPMAX*PFAC/FACA
C
      CALL PLTINI
C
      CALL GETCOLOR(ICOL0)
C
C---- re-origin for  Cp vs x  plot
      CALL PLOT(0.09 , 0.04 + CPMAX*PFAC + (YMAX-YMIN)*FACA, -3)
C
C---- plot Cp(x) axes
      CALL CPAXES(LCPGRD,
     &            N,X,Y,XOFA,YOFA,FACA,
     &            CPMIN,CPMAX,CPDEL,PFAC,CH,
     &            'XFOIL',VERSION)
C
C---- add displacement surface to airfoil if viscous flag is set
      IF(LVISC) CALL CPDISP(N,X,Y,NX,NY,XOFA,YOFA,FACA,
     &                      IVX,IBLTE,NBL,IPAN,DSTR,ANTE,ICOLS)
C
C---- add sonic Cp dashed line if within plot
      IF(CPSTAR.GE.CPMIN) CALL DASH(0.0,1.0,-CPSTAR*PFAC)
C
      CALL NEWPEN(2)

      ILE1 = IPAN(2,1)
      ILE2 = IPAN(2,2)
C
      IF(LVISC .AND. ILE1.GT.0 .AND. ILE2.GT.0) THEN
C----- plot viscous and inviscid Cp
       N1 = ILE1
       CALL NEWCOLOR(ICOLS(1))
       CALL XYLINE(N1,X(1),CPV(1),-XOFA,FACA,0.0,-PFAC,1)
C
       N2 = N - ILE2 + 1
       CALL NEWCOLOR(ICOLS(2))
       CALL XYLINE(N2,X(ILE2),CPV(ILE2),-XOFA,FACA,0.0,-PFAC,1)
C
       CALL NEWCOLOR(ICOL0)
       CALL XYLINE(NW,X(N+1),CPV(N+1),-XOFA,FACA,0.0,-PFAC,1)
C
       IF(LCPINV) THEN
        CALL NEWPEN(1)
        CALL CPDASH(N+NW,X,CPI,-XOFA,FACA,-PFAC)
       ENDIF
C
      ELSE
C----- plot inviscid Cp only
       CALL XYLINE(N,X,CPI,-XOFA,FACA,0.0,-PFAC,1)
      ENDIF
C
C
      IF(LCPREF) THEN
       CALL GETXYL(IQX,NCPREF,XPREF,CPREF,LABREF,
     &             'Enter Cp vs x data filename',OCNAME)
C
       CALL NEWCOLORNAME('cyan')
       CALL NEWPEN(2)
       DO K=1, NCPREF
         CALL PLSYMB((XPREF(K)+XOFA)*FACA,-PFAC*CPREF(K),
     &               SH,ISYM,0.0,0)
       ENDDO
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
C---- plot force coefficient
      YPLT = -CPMIN*PFAC
      CALL COEFPL(XPLT,YPLT,CH,LVISC,LFOREF,LVCONV,
     &            NAME,NNAME,
     &            REINF,MINF,ACRIT(1),ALFA,CL,CM,CD,CDP)
C
      IF(LFOREF) THEN
       CALL NEWCOLORNAME('cyan')
       YPLT = -CPMIN*PFAC
       CALL FOREF(XPLT,YPLT,CH,LVISC, MINF)
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
      IF(LCPREF .AND. NCPREF.GT.0 .AND. LABREF(1:1).NE.' ') THEN
       CALL NEWCOLORNAME('cyan')
       YPLT = YPLT - 3.5*CH
       CALL PLSYMB(XPLT-1.0*CH,YPLT+0.5*CH,SH,ISYM  ,0.0, 0)
       CALL PLCHAR(XPLT+1.0*CH,YPLT       ,CH,LABREF,0.0,-1)
       CALL NEWCOLOR(ICOL0)
      ENDIF
C

      CALL PLFLUSH
C
      RETURN
      END ! CPX




      SUBROUTINE UEX
C-----------------------------------------
C     Plots Ue vs x, integrated forces, 
C     parameters, and reference data.
C-----------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- set x location of label
      XPLT = 0.70
      IF(LFOREF) XPLT = 0.52
C
C---- size and type of reference-data symbol
      SH = 0.7*CH
      ISYM = 5
C
      UFAC = PLOTAR/(UEMAX-UEMIN)
C
C---- determine airfoil box size and location
      CALL AIRLIM(N,X,Y,XMIN,XMAX,YMIN,YMAX)
C
C---- y-offset for airfoil in  Cp vs x  plot
      FACA = FACAIR/(XMAX-XMIN)
      XOFA = XOFAIR*(XMAX-XMIN) - XMIN
      YOFA = YOFAIR*(XMAX-XMIN) - YMAX + UEMIN*UFAC/FACA
C
      CALL PLTINI
C
      CALL GETCOLOR(ICOL0)
C
C---- re-origin for  Ue vs x  plot
      CALL PLOT(0.09 , 0.04 - UEMIN*UFAC + (YMAX-YMIN)*FACA, -3)
C
C---- plot Ue(x) axes
      CALL UEAXES(LCPGRD,
     &            N,X,Y,XOFA,YOFA,FACA,
     &            UEMIN,UEMAX,UEDEL,UFAC,CH,
     &            'XFOIL',VERSION)
C
C---- add displacement surface to airfoil if viscous flag is set
      IF(LVISC) CALL CPDISP(N,X,Y,NX,NY,XOFA,YOFA,FACA,
     &                      IVX,IBLTE,NBL,IPAN,DSTR,ANTE,ICOLS)
C
C---- add sonic Cp dashed line if within plot
      IF(QSTAR.LE.UEMAX) CALL DASH(0.0,1.0,QSTAR*UFAC)
C
      CALL NEWPEN(2)
      IF(LVISC) THEN
C----- plot viscous and inviscid Ue
       ILE1 = IPAN(2,1)
       ILE2 = IPAN(2,2)
C
       N1 = ILE1
       CALL NEWCOLOR(ICOLS(1))
       CALL XYLINE(N1,X(1),QVIS(1),-XOFA,FACA,0.0,UFAC,1)
C
       N2 = N - ILE2 + 1
       CALL NEWCOLOR(ICOLS(2))
       CALL XYLINE(N2,X(ILE2),QVIS(ILE2),-XOFA,FACA,0.0,UFAC,1)
C
       CALL NEWCOLOR(ICOL0)
       CALL XYLINE(NW,X(N+1),QVIS(N+1),-XOFA,FACA,0.0,UFAC,1)
C
       CALL NEWPEN(1)
       CALL CPDASH(N+NW,X,QINV,-XOFA,FACA,UFAC)
      ELSE
C----- plot inviscid Cp only
       CALL XYLINE(N,X,QINV,-XOFA,FACA,0.0,UFAC,1)
      ENDIF
C
C
C---- plot force coefficient
      YPLT = UEMAX*UFAC
      CALL COEFPL(XPLT,YPLT,CH,LVISC,LFOREF,LVCONV,
     &            NAME,NNAME,
     &            REINF,MINF,ACRIT(1),ALFA,CL,CM,CD,CDP)
C
      IF(LFOREF) THEN
       CALL NEWCOLORNAME('cyan')
       YPLT = UEMAX*UFAC
       CALL FOREF(XPLT,YPLT,CH,LVISC, MINF)
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
      IF(LCPREF .AND. NCPREF.GT.0 .AND. LABREF(1:1).NE.' ') THEN
       CALL NEWCOLORNAME('cyan')
       YPLT = YPLT - 3.5*CH
       CALL PLSYMB(XPLT-1.0*CH,YPLT+0.5*CH,SH,ISYM  ,0.0, 0)
       CALL PLCHAR(XPLT+1.0*CH,YPLT       ,CH,LABREF,0.0,-1)
       CALL NEWCOLOR(ICOL0)
      ENDIF
C
      CALL PLFLUSH
C
      RETURN
      END ! UEX

 

      SUBROUTINE GETXYL(NDIM,N,X,Y,LABEL,PROMPT,FNAME)
C---------------------------------------------
C     Reads reference x,y data, with label
C---------------------------------------------
      DIMENSION X(NDIM), Y(NDIM)
      CHARACTER*(*) LABEL, PROMPT
      CHARACTER*(*) FNAME
C
      CHARACTER*80 FNNEW
C
      LU = 2
C
      N = 0
C
 1000 FORMAT(A)
 1100 FORMAT(/1X,A,1X,A)
      IF(FNAME.EQ.' ') THEN
       WRITE(*,1100) PROMPT
       READ(*,1000) FNAME
      ELSE
       WRITE(*,1100) PROMPT, FNAME
       READ(*,1000) FNNEW
       IF(FNNEW .NE. ' ') FNAME = FNNEW
      ENDIF
C
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=98)
C
C---- read first line for possible label
      READ(LU,1000) LABEL
C
      K1 = 1
      READ(LABEL,*,ERR=10) X(K1), Y(K1)
      K1 = 2
C
 10   DO K = K1, NDIM
        READ(LU,*,END=15,ERR=99) X(K), Y(K)
      ENDDO
 15   N = K-1
      CLOSE(LU)
C
      KP = INDEX(LABEL,'#')
      IF(KP.EQ.0) THEN
       CALL ASKS('Enter data description label^',LABEL)
      ELSE
       LABEL(KP:KP) = ' '
      ENDIF
C
      CALL STRIP(LABEL,NLABEL)
      RETURN
C
   98 WRITE(*,*) 'GETXYL: File OPEN error.'
      RETURN
C
   99 WRITE(*,*) 'GETXYL: File READ error.'
      CLOSE(LU)
      RETURN
C
      END ! GETXYL



      SUBROUTINE AIRLIM(N,X,Y,XMIN,XMAX,YMIN,YMAX)
      DIMENSION X(*),Y(*)
C-----------------------------------------
C     Sets airfoil width and thickness 
C     for airfoil plot space allocation.
C-----------------------------------------
C
      XMIN = X(1)
      XMAX = X(1)
      YMIN = Y(1)
      YMAX = Y(1)
      DO 4 I=1, N
        XMIN = MIN(XMIN,X(I))
        XMAX = MAX(XMAX,X(I))
        YMIN = MIN(YMIN,Y(I))
        YMAX = MAX(YMAX,Y(I))
 4    CONTINUE
      AIRDX = XMAX - XMIN
      AIRDY = YMAX - YMIN
C
C---- round up to nearest 10% of max dimension
      AIRDIM = MAX( AIRDX, AIRDY )
      AIRDX = 0.05*AIRDIM * AINT(AIRDX/(0.05*AIRDIM) + 1.2)
      AIRDY = 0.05*AIRDIM * AINT(AIRDY/(0.05*AIRDIM) + 1.2)
C
      XAVG = 0.5*(XMAX+XMIN)
      YAVG = 0.5*(YMAX+YMIN)
C
      XMIN = XAVG - 0.5*AIRDX
      XMAX = XAVG + 0.5*AIRDX
      YMIN = YAVG - 0.5*AIRDY
      YMAX = YAVG + 0.5*AIRDY
C
C---- fudge y-space again to 25% of plot width
      DDY = MIN( AIRDY , 0.25*AIRDX ) - AIRDY
C
C---- fudge y limits to match fudged y space, keeping average y the same
      YMIN = YMIN - 0.5*DDY
      YMAX = YMAX + 0.5*DDY
C
      RETURN
      END ! AIRLIM



      SUBROUTINE CPAXES(LGRID,
     &                  N,X,Y,XOFA,YOFA,FACA,
     &                  CPMIN,CPMAX,CPDEL,PFAC,CH,
     &                  CODE,VERSION)
C----------------------------------------------
C     Plots axes and airfoil for Cp vs x plot
C----------------------------------------------
      LOGICAL LGRID
      DIMENSION X(*),Y(*)
      CHARACTER*(*) CODE
C
      EXTERNAL PLCHAR
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- plot Cp axis from Cpmax to Cpmin
      CALL NEWPEN(2)
      CALL YAXIS(0.0,-CPMAX*PFAC,-(CPMIN-CPMAX)*PFAC,-CPDEL*PFAC,
     &           CPMAX,CPDEL,0.9*CH,1)
      CALL NEWPEN(3)
      YLAB = (FLOAT(INT(CPMIN/CPDEL + 0.01)/2) + 0.5)
     &     * (-CPDEL)*PFAC - 0.6*CH
      CALL PLCHAR(-4.0*CH,YLAB,1.4*CH,'C',0.0,1)
      CALL PLSUBS(-4.0*CH,YLAB,1.4*CH,'p',0.0,1,PLCHAR)
C
C---- plot Cp=0 line
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
C
C---- add tick marks
      DO 10 IT=0, 2
        XTIK = 0.5*FLOAT(IT)
        CALL PLOT((XTIK+XOFA)*FACA,0.005,3)
        CALL PLOT((XTIK+XOFA)*FACA,-.005,2)
   10 CONTINUE
C
      DO 15 IT=1, 9
        XTIK = 0.1*FLOAT(IT)
        CALL PLOT((XTIK+XOFA)*FACA,0.0025,3)
        CALL PLOT((XTIK+XOFA)*FACA,-.0025,2)
   15 CONTINUE
C
C---- plot airfoil contour
      CALL NEWPEN(2)
      CALL PLOT((X(1)+XOFA)*FACA,(Y(1)+YOFA)*FACA,3)
      DO 20 I=2, N
        CALL PLOT((X(I)+XOFA)*FACA,(Y(I)+YOFA)*FACA,2)
   20 CONTINUE
C
C---- plot code identifier
      CALL NEWPEN(2)
      CHI = 0.60*CH
      CHJ = 0.50*CH
      LENC = LEN(CODE)
      CALL PLCHAR(    CHI,-CPMIN*PFAC-1.0*CHI,CHI,CODE   ,0.0,LENC)
      CALL PLCHAR(    CHI,-CPMIN*PFAC-3.0*CHI,CHJ,'V'    ,0.0,1)
      CALL PLNUMB(3.0*CHJ,-CPMIN*PFAC-3.0*CHI,CHJ,VERSION,0.0,2)
C
      IF(LGRID) THEN
       X0 = XOFA*FACA
       Y0 = -CPMAX*PFAC
       NXG = 10
       NYG = INT((CPMIN-CPMAX)/CPDEL + 0.01) * 5
       DXG = 0.1*FACA 
       DYG = -CPDEL*PFAC / 5.0
       CALL NEWPEN(1)
       CALL PLGRID(X0,Y0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      RETURN
      END ! CPAXES



      SUBROUTINE UEAXES(LGRID,
     &                  N,X,Y,XOFA,YOFA,FACA,
     &                  UEMIN,UEMAX,UEDEL,UFAC,CH,
     &                  CODE,VERSION)
C----------------------------------------------
C     Plots axes and airfoil for Cp vs x plot
C----------------------------------------------
      LOGICAL LGRID
      DIMENSION X(*),Y(*)
      CHARACTER*(*) CODE
C
      EXTERNAL PLCHAR
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- plot Cp axis from Cpmax to Cpmin
      CALL NEWPEN(2)
      CALL YAXIS(0.0,UEMIN*UFAC,(UEMAX-UEMIN)*UFAC,UEDEL*UFAC,
     &           UEMIN,UEDEL,0.9*CH,1)
      CALL NEWPEN(3)
      YLAB = (FLOAT(INT(UEMAX/UEDEL + 0.01)/2) + 0.5)
     &     * (UEDEL)*UFAC - 0.6*CH
      CALL PLCHAR(-4.0*CH,YLAB,1.2*CH,'U',0.0,1)
      CALL PLSUBS(-4.0*CH,YLAB,1.2*CH,'e',0.0,1,PLCHAR)
C
C---- plot Cp=0 line
      CALL NEWPEN(1)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(1.0,0.0,2)
C
C---- add tick marks
      DO 10 IT=0, 2
        XTIK = 0.5*FLOAT(IT)
        CALL PLOT((XTIK+XOFA)*FACA,0.005,3)
        CALL PLOT((XTIK+XOFA)*FACA,-.005,2)
   10 CONTINUE
C
      DO 15 IT=1, 9
        XTIK = 0.1*FLOAT(IT)
        CALL PLOT((XTIK+XOFA)*FACA,0.0025,3)
        CALL PLOT((XTIK+XOFA)*FACA,-.0025,2)
   15 CONTINUE
C
C---- plot airfoil contour
      CALL NEWPEN(2)
      CALL PLOT((X(1)+XOFA)*FACA,(Y(1)+YOFA)*FACA,3)
      DO 20 I=2, N
        CALL PLOT((X(I)+XOFA)*FACA,(Y(I)+YOFA)*FACA,2)
   20 CONTINUE
C
C---- plot code identifier
      CALL NEWPEN(2)
      CHI = 0.60*CH
      CHJ = 0.50*CH
      LENC = LEN(CODE)
      CALL PLCHAR(    CHI,UEMAX*UFAC-1.0*CHI,CHI,CODE   ,0.0,LENC)
      CALL PLCHAR(    CHI,UEMAX*UFAC-3.0*CHI,CHJ,'V'    ,0.0,1)
      CALL PLNUMB(3.0*CHJ,UEMAX*UFAC-3.0*CHI,CHJ,VERSION,0.0,2)
C
      IF(LGRID) THEN
       X0 = XOFA*FACA
       Y0 = UEMIN*UFAC
       NXG = 10
       NYG = INT((UEMAX-UEMIN)/UEDEL + 0.01) * 5
       DXG = 0.1*FACA 
       DYG = UEDEL*UFAC / 5.0
       CALL NEWPEN(1)
       CALL PLGRID(X0,Y0, NXG,DXG, NYG,DYG, LMASK2 )
      ENDIF
C
      RETURN
      END ! UEAXES



      SUBROUTINE CPDISP(N,X,Y,NX,NY,XOFA,YOFA,FACA,
     &                  IVX,IBLTE,NBL,IPAN,DSTR,ANTE,ICOLS)
C----------------------------------------------
C     Plots displacement surface on airfoil
C----------------------------------------------
      REAL NX,NY
      DIMENSION X(*),Y(*),NX(*),NY(*)
      DIMENSION IBLTE(2),NBL(2),IPAN(IVX,2)
      DIMENSION DSTR(IVX,2)
      DIMENSION ICOLS(2)
C
      IF(IBLTE(1) .EQ. 0 .OR.
     &   IBLTE(2) .EQ. 0      ) THEN
       RETURN
      ENDIF

      CALL GETCOLOR(ICOL0)
      CALL NEWPEN(1)
C
C---- plot displacement surface on both airfoil sides
      DO 40 IS=1, 2
        IPEN = 3
        DO 410 IBL=2, IBLTE(IS)
          I = IPAN(IBL,IS)
          XPLT = X(I) + NX(I)*DSTR(IBL,IS)
          YPLT = Y(I) + NY(I)*DSTR(IBL,IS)
          CALL NEWCOLOR(ICOLS(IS))
          CALL PLOT((XPLT+XOFA)*FACA,(YPLT+YOFA)*FACA,IPEN)
          IPEN = 2
  410   CONTINUE
   40 CONTINUE
C
      IS = 2
C
C---- set upper and lower wake Dstar fractions based on first wake point
      DSTRTE = DSTR(IBLTE(IS)+1,IS)
      IF(DSTRTE.NE.0.0) THEN
       DSF1 = (DSTR(IBLTE(1),1) + 0.5*ANTE) / DSTRTE
       DSF2 = (DSTR(IBLTE(2),2) + 0.5*ANTE) / DSTRTE
      ELSE
       DSF1 = 0.5
       DSF2 = 0.5
      ENDIF
C
C---- plot upper wake displacement surface
ccc      CALL NEWCOLOR(ICOLS(1))
      CALL NEWCOLOR(ICOL0)
      IBL = IBLTE(1)
      I = IPAN(IBL,1)
      XPLT = X(I) + NX(I)*DSTR(IBL,1)
      YPLT = Y(I) + NY(I)*DSTR(IBL,1)
      CALL PLOT((XPLT+XOFA)*FACA,(YPLT+YOFA)*FACA,3)
      DO 50 IBL=IBLTE(IS)+1, NBL(IS)
        I = IPAN(IBL,IS)
        XPLT = X(I) - NX(I)*DSTR(IBL,IS)*DSF1
        YPLT = Y(I) - NY(I)*DSTR(IBL,IS)*DSF1
        CALL PLOT((XPLT+XOFA)*FACA,(YPLT+YOFA)*FACA,2)
   50 CONTINUE
C
C---- plot lower wake displacement surface
ccc      CALL NEWCOLOR(ICOLS(2))
      CALL NEWCOLOR(ICOL0)
      IBL = IBLTE(2)
      I = IPAN(IBL,2)
      XPLT = X(I) + NX(I)*DSTR(IBL,2)
      YPLT = Y(I) + NY(I)*DSTR(IBL,2)
      CALL PLOT((XPLT+XOFA)*FACA,(YPLT+YOFA)*FACA,3)
      DO 55 IBL=IBLTE(IS)+1, NBL(IS)
        I = IPAN(IBL,IS)
        XPLT = X(I) + NX(I)*DSTR(IBL,IS)*DSF2
        YPLT = Y(I) + NY(I)*DSTR(IBL,IS)*DSF2
        CALL PLOT((XPLT+XOFA)*FACA,(YPLT+YOFA)*FACA,2)
   55 CONTINUE
C
      CALL PLFLUSH
      CALL NEWCOLOR(ICOL0)
C
      RETURN
      END ! CPDISP



      SUBROUTINE CPDASH(N,X,Y, XOFA,FACA,YFAC)
C----------------------------------
C     Plot dashed y(x) distribution.
C----------------------------------
      DIMENSION X(*),Y(*)
C
      DO 40 I=2, N
        DX = X(I) - X(I-1)
        DY = Y(I) - Y(I-1)
        CALL PLOT((X(I)-0.75*DX-XOFA)*FACA,YFAC*(Y(I)-0.75*DY),3)
        CALL PLOT((X(I)-0.25*DX-XOFA)*FACA,YFAC*(Y(I)-0.25*DY),2)
   40 CONTINUE
C
      RETURN
      END ! CPDASH
 

      SUBROUTINE SEQLAB(XLAB,YLAB,XL1,XL2,XL3,XL4,XL5,XL6,
     &                  CHSEQ,IPAR,LVT)
C-------------------------------------------------------------
C     Plots label for alpha- or CL-sequence  Cp vs x  plot.
C-------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LVT
C
      EXTERNAL PLCHAR
C
      CHN = 1.10*CH
      CCH = 0.90*CH
      CHS = 0.70*CH
C
      YSPACE = 2.1*CCH
C
C---- x-location of parameter labels
      XLP = XLAB + 1.0*CCH
      IF(LVT) XLP = XLAB + 7.0*CCH
C
      IF(IPAR.EQ.1) THEN
C
C----- plot case name
       CALL NEWPEN(3)
       YLAB = YLAB - CH
       XPLT = XLP + 8.0*CCH - 0.5*CHN*FLOAT(NNAME)
       CALL PLCHAR(XPLT,YLAB,CHN,NAME,0.0,NNAME)
C
       YLAB = YLAB - YSPACE
       CALL NEWPEN(3)
       IF    (MATYP.EQ.1) THEN
        CALL PLCHAR(XLP,YLAB,CCH,'  Ma = ',0.0,7)
       ELSEIF(MATYP.EQ.2) THEN
ccc     CALL PLMATH(XLP,YLAB,CCH,'   _   ',0.0,7)
        CALL PLCHAR(XLP,YLAB,CCH,'Ma C = ',0.0,7)
        CALL PLMATH(XLP,YLAB,CCH,'  R    ',0.0,7)
        CALL PLSUBS(XLP+3.0*CCH,YLAB,CCH,'L',0.0,1,PLCHAR)
       ELSEIF(MATYP.EQ.3) THEN
        CALL PLCHAR(XLP,YLAB,CCH,'Ma C = ',0.0,7)
        CALL PLSUBS(XLP+3.0*CCH,YLAB,CCH,'L',0.0,1,PLCHAR)
       ENDIF
       CALL PLNUMB(XLP+7.0*CCH,YLAB,CCH, MINF1,0.0,3)
C
       IF(LVT) THEN
        YLAB = YLAB - YSPACE
        CALL NEWPEN(3)
        IF    (RETYP.EQ.1) THEN
         CALL PLCHAR(XLP,YLAB,CCH,'  Re = ',0.0,7)
        ELSEIF(RETYP.EQ.2) THEN
ccc      CALL PLMATH(XLP,YLAB,CCH,'   _   ',0.0,7)
         CALL PLCHAR(XLP,YLAB,CCH,'Re C = ',0.0,7)
         CALL PLMATH(XLP,YLAB,CCH,'  R    ',0.0,7)
         CALL PLSUBS(XLP+3.0*CCH,YLAB,CCH,'L',0.0,1,PLCHAR)
        ELSEIF(RETYP.EQ.3) THEN
         CALL PLCHAR(XLP,YLAB,CCH,'Re C = ',0.0,7)
         CALL PLSUBS(XLP+3.0*CCH,YLAB,CCH,'L',0.0,1,PLCHAR)
        ENDIF
        NDIG = 3
        IF(REINF .GE. 9.9995E6) NDIG = 2
        IF(REINF .GE. 99.995E6) NDIG = 1
        IF(REINF .GE. 999.95E6) NDIG = 0
        RE6 = REINF1*1.0E-6
        CALL PLNUMB(XLP+ 7.0*CCH,YLAB        ,     CCH,RE6  ,0.0,NDIG)
        CALL PLMATH(XLP+12.1*CCH,YLAB+0.2*CCH,0.80*CCH,'#'  ,0.0,1)
        CALL PLCHAR(XLP+13.0*CCH,YLAB        ,     CCH,'10' ,0.0,2)
        CALL PLMATH(XLP+15.0*CCH,YLAB        ,1.10*CCH,  '6',0.0,1)
C
        YLAB = YLAB - YSPACE
        CALL NEWPEN(3)
        CALL PLCHAR(XLP        ,YLAB,CCH,'  N  = ',0.0,7)
        CALL PLSUBS(XLP+2.0*CCH,YLAB,CCH,   'cr'  ,0.0,2,PLCHAR)
        CALL PLNUMB(XLP+7.0*CCH,YLAB,CCH,ACRIT(1) ,0.0,3)
        IF(ACRIT(1) .NE. ACRIT(2)) THEN
         CALL PLNUMB(XLP+14.0*CCH,YLAB,CCH,ACRIT(2) ,0.0,3)
        ENDIF
       ENDIF
C
      ENDIF
C
      XL1 = XLAB
      XL2 = XL1 + 7.0*CHS
      XL3 = XL2 + 8.0*CHS
      XL4 = XL3 + 8.0*CHS
      XL5 = XL4 + 9.0*CHS
      XL6 = XL5 + 7.0*CHS
      YLAB = YLAB - 2.7*CHS
      CALL NEWPEN(3)
      CALL PLMATH(XL1+2.0*CHS,YLAB,1.3*CHS,'a',0.0,1)
      CALL PLCHAR(XL2+2.0*CHS,YLAB,CHS,'C',0.0,1)
      CALL PLSUBS(XL2+2.0*CHS,YLAB,CHS,'L',0.0,1,PLCHAR)
      CALL PLCHAR(XL3+2.0*CHS,YLAB,CHS,'C',0.0,1)
      CALL PLSUBS(XL3+2.0*CHS,YLAB,CHS,'M',0.0,1,PLCHAR)
      IF(LVT) THEN
       CALL PLCHAR(XL4+2.5*CHS,YLAB,    CHS,'C',0.0,1)
       CALL PLSUBS(XL4+2.5*CHS,YLAB,    CHS,'D',0.0,1,PLCHAR)
       CALL PLCHAR(XL5        ,YLAB,0.8*CHS,'Top',0.0,3)
       CALL PLCHAR(XL5+3.0*CHS,YLAB,    CHS,'X'  ,0.0,1)
       CALL PLCHAR(XL5+3.9*CHS,YLAB,0.6*CHS,'tr' ,0.0,2)
       CALL PLCHAR(XL6        ,YLAB,0.8*CHS,'Bot',0.0,3)
       CALL PLCHAR(XL6+3.0*CHS,YLAB,    CHS,'X'  ,0.0,1)
       CALL PLCHAR(XL6+3.9*CHS,YLAB,0.6*CHS,'tr' ,0.0,2)
      ENDIF
C
      CALL NEWPEN(1)
      CALL PLOT(XL1        ,YLAB-0.6*CHS,3)
      CALL PLOT(XL1+5.0*CHS,YLAB-0.6*CHS,2)
      CALL PLOT(XL2        ,YLAB-0.6*CHS,3)
      CALL PLOT(XL2+6.0*CHS,YLAB-0.6*CHS,2)
      CALL PLOT(XL3        ,YLAB-0.6*CHS,3)
      CALL PLOT(XL3+6.0*CHS,YLAB-0.6*CHS,2)
      IF(LVT) THEN
       CALL PLOT(XL4        ,YLAB-0.6*CHS,3)
       CALL PLOT(XL4+7.0*CHS,YLAB-0.6*CHS,2)
       CALL PLOT(XL5        ,YLAB-0.6*CHS,3)
       CALL PLOT(XL5+5.0*CHS,YLAB-0.6*CHS,2)
       CALL PLOT(XL6        ,YLAB-0.6*CHS,3)
       CALL PLOT(XL6+5.0*CHS,YLAB-0.6*CHS,2)
      ENDIF
C
      YLAB = YLAB - 0.5*CHS
C
      CHSEQ = CHS
      RETURN
      END ! SEQLAB


      SUBROUTINE SEQPLT(YLAB,XL1,XL2,XL3,XL4,XL5,XL6,
     &                  CHS,ALT,CLT,CMT,LVT)
C------------------------------------------------
C     Plots force coefficients for one point on
C     alpha- or CL-sequence  Cp vs x  plot.
C------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LVT
C
      CALL NEWPEN(2)
      DXL1 = 0.
      DXL2 = 0.
      DXL3 = CHS
      DXL4 = 0.
      IF(ALT .LT. 0.0) DXL1 = DXL1 - CHS
      IF(CLT .LT. 0.0) DXL2 = DXL2 - CHS
      IF(CMT .LT. 0.0) DXL3 = DXL3 - CHS
      IF(CD  .LT. 0.0) DXL4 = DXL4 - CHS
      IF(ALT .GE. 10.) DXL1 = DXL1 - CHS
      IF(ALT .LE.-10.) DXL1 = DXL1 - CHS
C
      YLAB = YLAB - 2.1*CHS
      CALL PLNUMB(XL1+DXL1,YLAB,CHS,ALT,0.0,3)
      CALL PLNUMB(XL2+DXL2,YLAB,CHS,CLT,0.0,4)
      CALL PLNUMB(XL3+DXL3,YLAB,CHS,CMT,0.0,3)
      IF(LVT) THEN
       CALL PLNUMB(XL4+DXL4,YLAB,CHS,      CD,0.0,5)
       CALL PLNUMB(XL5     ,YLAB,CHS,XOCTR(1),0.0,3)
       CALL PLNUMB(XL6     ,YLAB,CHS,XOCTR(2),0.0,3)
      ENDIF
C
      RETURN
      END ! SEQPLT




      SUBROUTINE COEFPL(XL,YL,CH,LVISC,LFOREF,LVCONV,
     &                  NAME,NNAME,
     &                  REINF,MINF,ACRIT,ALFA,CL,CM,CD,CDP)
C------------------------------------------------------------------
C     Plots force coefficients for single-point  Cp vs x  plot.
C
C     XL,YL   upper-left corner of label block, 
C             returned as location of lower-left corner
C
C------------------------------------------------------------------
      LOGICAL LVISC, LFOREF, LVCONV
      CHARACTER*(*) NAME
      REAL MINF
      REAL ACRIT(*)
C
      EXTERNAL PLCHAR
C
      CHN = 1.10*CH
      CCH = 0.90*CH
      SCH = 0.70*CH
C
      YSPACE = 2.2*CCH
C
      ADEG = ALFA * 45.0/ATAN(1.0)
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWPEN(3)
      XPLT1 = XL + 16.0*CCH -     FLOAT(NNAME)*CHN
      XPLT2 = XL +  6.0*CCH - 0.5*FLOAT(NNAME)*CHN
      IF(     LFOREF) XPLT = MIN( XPLT1 , XPLT2 )
      IF(.NOT.LFOREF) XPLT = XPLT2
      YL = YL - CHN
      CALL PLCHAR(XPLT,YL,CHN,NAME,0.0,NNAME)
C
      YL = YL - 0.2*CH
      CALL NEWPEN(2)
C
      IF(MINF .GT. 0.0) THEN
       YL = YL - 2.0*CH
       CALL PLCHAR(XL        ,YL,CCH,'Ma = ',0.0,5)
       CALL PLNUMB(XL+5.0*CCH,YL,CCH, MINF  ,0.0,4)
      ENDIF
C
      IF(LVISC) THEN
       YL = YL - YSPACE
       CALL PLCHAR(XL         ,YL         ,CCH,'Re = '   ,0.0,5)
       NDIG = 3
       IF(REINF .GE. 9.9995E6) NDIG = 2
       IF(REINF .GE. 99.995E6) NDIG = 1
       IF(REINF .GE. 999.95E6) NDIG = 0
       CALL PLNUMB(XL+ 5.0*CCH,YL         ,CCH, REINF*1.E-6,0.0,NDIG)
       CALL PLMATH(XL+10.1*CCH,YL+0.10*CCH,0.80*CCH,'#'  ,0.0,1)
       CALL PLCHAR(XL+10.9*CCH,YL         ,     CCH,'10' ,0.0,2)
       CALL PLMATH(XL+12.9*CCH,YL         ,1.10*CCH,  '6',0.0,1)
      ENDIF
C
      YL = YL - YSPACE
      CALL PLMATH(XL        ,YL,1.2*CCH,'a',0.0,1)
      CALL PLMATH(XL        ,YL,CCH,'   = ',0.0,5)
      CALL PLNUMB(XL+5.0*CCH,YL,CCH, ADEG  ,0.0,4)
      CALL PLMATH(999.0     ,YL,CCH,'"'    ,0.0,1)
C
      YL = YL - YSPACE
      CALL PLCHAR(XL        ,YL,CCH,'C  = ',0.0,5)
      CALL PLSUBS(XL        ,YL,CCH, 'L'   ,0.0,1,PLCHAR)
      CALL PLNUMB(XL+5.0*CCH,YL,CCH, CL    ,0.0,4)
C
      YL = YL - YSPACE
      CALL PLCHAR(XL        ,YL,CCH,'C  = ',0.0,5)
      CALL PLSUBS(XL        ,YL,CCH, 'M'   ,0.0,1,PLCHAR)
      CALL PLNUMB(XL+5.0*CCH,YL,CCH, CM    ,0.0,4)
C
      IF(.NOT.LVISC) THEN
       YL = YL - YSPACE
       CALL PLCHAR(XL        ,YL,CCH,'C  = ',0.0,5)
       CALL PLSUBS(XL        ,YL,CCH, 'Dp'  ,0.0,2,PLCHAR)
       CALL PLNUMB(XL+5.0*CCH,YL,CCH, CDP   ,0.0,5)
      ENDIF
C
      IF(LVISC) THEN
       YL = YL - YSPACE
       CALL PLCHAR(XL        ,YL,CCH,'C  = ',0.0,5)
       CALL PLSUBS(XL        ,YL,CCH, 'D'   ,0.0,1,PLCHAR)
       CALL PLNUMB(XL+5.0*CCH,YL,CCH, CD    ,0.0,5)
C
       ELOD = 0.
       IF(CD.NE.0.0) ELOD = CL/CD
C
       YL = YL - YSPACE
       CALL PLCHAR(XL        ,YL,0.8*CCH,'L/D',0.0,3)
       CALL PLCHAR(XL        ,YL,CCH,'   = ',0.0,5)
       CALL PLNUMB(XL+5.0*CCH,YL,CCH, ELOD  ,0.0,2)
C
       YL = YL - YSPACE
       CALL PLCHAR(XL        ,YL,CCH,'N  = ',0.0,5)
       CALL PLSUBS(XL        ,YL,CCH, 'cr'  ,0.0,2,PLCHAR)
       CALL PLNUMB(XL+5.0*CCH,YL,CCH, ACRIT(1) ,0.0,2)
       IF(ACRIT(1) .NE. ACRIT(2)) THEN
       CALL PLNUMB(XL+11.0*CCH,YL,CCH, ACRIT(2) ,0.0,2)
       ENDIF
C
      ENDIF
C
      IF(LVISC .AND. .NOT.LVCONV) THEN
       CALL NEWCOLORNAME('red')
       YL = YL - 3.0*CCH
       CALL PLCHAR(XL-5.0*CCH,YL,1.5*CCH,'* NOT CONVERGED *',0.0,17)
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
C
      RETURN
      END ! COEFPL



      SUBROUTINE FOREF(XL,YL,CH,LVISC, MINF )
C---------------------------------------------
C     Plots reference data force coefficients
C     next to calculated coefficients.
C
C     XL,YL   upper-left corner of label block, 
C             returned as location of lower-left corner
C
C---------------------------------------------
      LOGICAL LVISC
      REAL MINF
C
      CHARACTER*32 LABEXP
C
      CHN = 1.10*CH
      CCH = 0.90*CH
C
      YSPACE = 2.2*CCH
C
      XL0 = XL
      YL0 = YL
C
      CALL PLFLUSH
 10   WRITE(*,*) 'Enter reference  Mach, Re, Alpha, CL, CD, CM:'
      READ(*,*,ERR=10) AMEX, REEX, ALEX, CLEX, CDEX, CMEX
C
      XL = XL + 18.5*CCH
      YL = YL - CHN
C
      YL = YL - 0.2*CH
      CALL NEWPEN(2)
C
      IF(MINF .GT. 0.0) THEN
       YL = YL - YSPACE
       CALL PLNUMB(XL,YL,CCH,AMEX,0.0,3)
      ENDIF
C
      IF(LVISC) THEN
       YL = YL - YSPACE
       CALL PLNUMB(XL,YL,CCH,REEX*1.0E-6,0.0,3)
       CALL PLMATH(XL+5.0*CCH,YL+0.10*CCH,0.80*CCH,'#'  ,0.0,1)
       CALL PLCHAR(XL+5.8*CCH,YL         ,     CCH,'10' ,0.0,2)
       CALL PLMATH(XL+7.8*CCH,YL         ,1.10*CCH,  '6',0.0,1)
      ENDIF
C
      YL = YL - YSPACE
      CALL PLNUMB(XL,YL,CCH,ALEX,0.0,3)
C
      YL = YL - YSPACE
      CALL PLNUMB(XL,YL,CCH,CLEX,0.0,4)
C
      YL = YL - YSPACE
      CALL PLNUMB(XL,YL,CCH,CMEX,0.0,4)
C
      IF(LVISC) THEN
       YL = YL - YSPACE
       CALL PLNUMB(XL,YL,CCH,CDEX,0.0,5)
C
       YL = YL - YSPACE
       ELOD = 0.0
       IF(CDEX.NE.0.0) ELOD = CLEX/CDEX
       CALL PLNUMB(XL,YL,CCH,ELOD,0.0,2)
      ENDIF
C
      CALL NEWPEN(1)
      XLIN = XL - 1.5*CCH
      CALL PLOT(XLIN,YL0,3)
      CALL PLOT(XLIN,YL ,2)
C
      CALL PLFLUSH
C
      CALL ASKS('Enter reference force data label^',LABEXP)
      CALL NEWPEN(3)
      YL1 = YL0 - CHN
      CALL PLCHAR(XL,YL1,0.9*CHN,LABEXP,0.0,-1)
C
      RETURN
      END ! FOREF



      SUBROUTINE CPVEC
C-------------------------------------------------------
C     Plots airfoil with normal pressure force vectors.
C-------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      DO 2 I=1, N
        W1(I) = X(I)
        W2(I) = Y(I)
 2    CONTINUE
C
      CALL ROTATE(W1,W2,N,ALFA)
      CALL NCALC(W1,W2,S,N,W3,W4)
C
C---- set geometric limits
      XMIN = W1(1)
      XMAX = W1(1)
      YMIN = W2(1)
      YMAX = W2(1)
      DO 5 I=1, N
        XMIN = MIN(XMIN,W1(I))
        XMAX = MAX(XMAX,W1(I))
        YMIN = MIN(YMIN,W2(I))
        YMAX = MAX(YMAX,W2(I))
 5    CONTINUE
C
C---- set pressure vector scale VSF
      XRANGE = MAX(1.0E-9, XMAX-XMIN)
      YRANGE = MAX(1.0E-9, YMAX-YMIN)
      VSF = VFAC / MIN( 1.0/XRANGE , PLOTAR/YRANGE )
C
C
C---- set limits again, including pressure vectors
      DO 8 I=1, N
        IF(     LVISC) CP = CPV(I)
        IF(.NOT.LVISC) CP = CPI(I)
        DX = ABS(CP)*VSF*W3(I)
        DY = ABS(CP)*VSF*W4(I)
        XMIN = MIN(XMIN,W1(I)+DX)
        XMAX = MAX(XMAX,W1(I)+DX)
        YMIN = MIN(YMIN,W2(I)+DY)
        YMAX = MAX(YMAX,W2(I)+DY)
 8    CONTINUE
C
C---- set scale, offsets, to center airfoil+vectors in plot area
      XRANGE = MAX(1.0E-9, XMAX-XMIN)
      YRANGE = MAX(1.0E-9, YMAX-YMIN)

      GSF = MIN( 1.0/XRANGE , PLOTAR/YRANGE )
      XOFG = XMIN - 0.5*(1.0   -GSF*XRANGE)/GSF - 0.05/GSF
      YOFG = YMIN - 0.5*(PLOTAR-GSF*YRANGE)/GSF - 0.05/GSF
C
      CALL PLTINI
C
      CALL NEWPEN(2)
      CALL PLOT((W1(1)-XOFG)*GSF,(W2(1)-YOFG)*GSF,3)
      DO 10 I=2, N
        CALL PLOT((W1(I)-XOFG)*GSF,(W2(I)-YOFG)*GSF,2)
   10 CONTINUE
C
      DO 20 I=2, N-1
        IF(     LVISC) CP = CPV(I)
        IF(.NOT.LVISC) CP = CPI(I)
        DX = -CP*VSF*W3(I)*GSF
        DY = -CP*VSF*W4(I)*GSF
        XL = (W1(I)-XOFG)*GSF
        YL = (W2(I)-YOFG)*GSF
        IF(CP.LT.0.0) CALL ARROW(XL   ,YL   ,DX,DY)
        IF(CP.GE.0.0) CALL ARROW(XL-DX,YL-DY,DX,DY)
   20 CONTINUE
C
      CALL PLFLUSH
      RETURN
      END ! CPVEC



      SUBROUTINE CPVECI
C-------------------------------------------------------
C     Plots airfoil with normal pressure force vectors.
C-------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C

 2000 format(
     & /'Begin %I MLine'
     & /'%I b 65535'
     & /'0 0 0 [] 0 SetB'
     & /'%I cfg Black'
     & /'0 0 0 SetCFg'
     & /'%I cbg White'
     & /'1 1 1 SetCBg'
     & /'none SetP %I p n'
     & /'%I t'
     & /'[ 0.1 -0 -0 0.1 0 0 ] concat'
     & /'%I', i3)

 2100 format(1x,2i9)

 3000 format(
     &  i3,' MLine'
     & /'%I 1'
     & /'End' )


      DO 2 I=1, N
        W1(I) = X(I)
        W2(I) = Y(I)
 2    CONTINUE
C
      CALL ROTATE(W1,W2,N,ALFA)
      CALL NCALC(W1,W2,S,N,W3,W4)
C
C---- set geometric limits
      XMIN = W1(1)
      XMAX = W1(1)
      YMIN = W2(1)
      YMAX = W2(1)
      DO 5 I=1, N
        XMIN = MIN(XMIN,W1(I))
        XMAX = MAX(XMAX,W1(I))
        YMIN = MIN(YMIN,W2(I))
        YMAX = MAX(YMAX,W2(I))
 5    CONTINUE
C
      vfaci = 0.08

C---- set pressure vector scale VSF
      XRANGE = MAX(1.0E-9, XMAX-XMIN)
      YRANGE = MAX(1.0E-9, YMAX-YMIN)
      VSF = VFACI / MIN( 1.0/XRANGE , PLOTAR/YRANGE )
C
C
C---- set limits again, including pressure vectors
      DO 8 I=1, N
        IF(     LVISC) CP = CPV(I)
        IF(.NOT.LVISC) CP = CPI(I)
        DX = ABS(CP)*VSF*W3(I)
        DY = ABS(CP)*VSF*W4(I)
        XMIN = MIN(XMIN,W1(I)+DX)
        XMAX = MAX(XMAX,W1(I)+DX)
        YMIN = MIN(YMIN,W2(I)+DY)
        YMAX = MAX(YMAX,W2(I)+DY)
 8    CONTINUE
C
      write(*,*) xmin, xmax
      write(*,*) ymin, ymax
      write(*,*) 'enter xmin,xmax, ymin,ymax'
      read(*,*) xmin,xmax, ymin,ymax

C---- set scale, offsets, to center airfoil+vectors in plot area
      XRANGE = MAX(1.0E-9, XMAX-XMIN)
      YRANGE = MAX(1.0E-9, YMAX-YMIN)

      GSF = MIN( 1.0/XRANGE , PLOTAR/YRANGE )
      gsfi = gsf * 10000.0

      XOFG = XMIN - 0.5*(1.0   -GSF*XRANGE)/GSF - 0.05/GSF
      YOFG = YMIN - 0.5*(PLOTAR-GSF*YRANGE)/GSF - 0.05/GSF
C
      CALL PLTINI
C
      CALL NEWPEN(2)
      CALL PLOT((W1(1)-XOFG)*GSF,(W2(1)-YOFG)*GSF,3)
      DO 10 I=2, N
        CALL PLOT((W1(I)-XOFG)*GSF,(W2(I)-YOFG)*GSF,2)
   10 CONTINUE
C

      lu = 1

      write(lu,2000) n
      do i = 1, n
        write(lu,2100) INT((W1(I)-XOFG)*GSFI), INT((W2(I)-YOFG)*GSFI)
      enddo
      write(lu,3000) n


      DO 20 I=2, N-1
        IF(     LVISC) CP = CPV(I)
        IF(.NOT.LVISC) CP = CPI(I)
        DX = -CP*VSF*W3(I)*GSFI
        DY = -CP*VSF*W4(I)*GSFI
        XL = (W1(I)-XOFG)*GSFI
        YL = (W2(I)-YOFG)*GSFI
        IF(CP.LT.0.0) THEN
         X0 = XL
         Y0 = YL
        ELSE
         X0 = XL - DX
         Y0 = YL - DY
        ENDIF
        X1 = X0 + 0.85*DX + 0.02*DY
        Y1 = Y0 + 0.85*DY - 0.02*DX
        X2 = X0 + 0.85*DX - 0.02*DY
        Y2 = Y0 + 0.85*DY + 0.02*DX
        write(lu,2000) 5
        write(lu,2100) INT(x0), INT(y0)
        write(lu,2100) INT(x0+dx), INT(y0+dy)
        write(lu,2100) INT(x1), INT(y1)
        write(lu,2100) INT(x2), INT(y2)
        write(lu,2100) INT(x0+dx), INT(y0+dy)
        write(lu,3000) 5

      CALL PLOT(X0*gsf/gsfi,Y0*gsf/gsfi,3)
      CALL PLOT((X0+DX)*gsf/gsfi,(Y0+DY)*gsf/gsfi,2)
      CALL PLOT(X1*gsf/gsfi,Y1*gsf/gsfi,2)
      CALL PLOT(X2*gsf/gsfi,Y2*gsf/gsfi,2)
      CALL PLOT((X0+DX)*gsf/gsfi,(Y0+DY)*gsf/gsfi,2)


   20 CONTINUE

      CALL PLFLUSH
C
      RETURN
      END ! CPVECI


      SUBROUTINE PPAPLT(NPPAI,IPPAI)
      DIMENSION IPPAI(*)
C-------------------------------------------
C     Plots mutiple polar airfoils overlaid
C-------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
      INCLUDE 'XDES.INC'
C
      CALL PLTINI
      CALL GOFINI
C
      CALL NEWPEN(1)
C
      IF(LGGRID) THEN
C------ plot outline
ccc     CALL PLOT(XMOD(XGMIN),YMOD(YGMIN),3)
        CALL PLOT(XMOD(XGMAX),YMOD(YGMIN),3)
        CALL PLOT(XMOD(XGMAX),YMOD(YGMAX),2)
        CALL PLOT(XMOD(XGMIN),YMOD(YGMAX),2)
ccc     CALL PLOT(XMOD(XGMIN),YMOD(YGMIN),2)
C
        CALL XAXIS(XMOD(XGMIN),YMOD(YGMIN),(XGMAX-XGMIN)*XSF,
     &             DXYG*XSF, XGMIN,DXYG,CHG,-2)
        CALL YAXIS(XMOD(XGMIN),YMOD(YGMIN),(YGMAX-YGMIN)*YSF,
     &             DXYG*YSF, YGMIN,DXYG,CHG,-2)
C
C------ fine grid
        NXG = INT((XGMAX-XGMIN)/DXYG + 0.01)
        NYG = INT((YGMAX-YGMIN)/DXYG + 0.01)
        X0 = XMOD(XGMIN)
        Y0 = YMOD(YGMIN)
        DXG = (XMOD(XGMAX)-X0)/NXG
        DYG = (YMOD(YGMAX)-Y0)/NYG
        CALL PLGRID(X0,Y0,NXG,DXG,NYG,DYG, LMASK2)
C
        XL0 = XMOD(XGMIN) + 1.0*CH
        YL0 = YMOD(YGMAX) + 3.0*CH
      ELSE
C
C------ plot chord line and tick marks every 10% chord
        CALL PLOT(XMOD(0.0),YMOD(0.0),3)
        CALL PLOT(XMOD(1.0),YMOD(0.0),2)
        DO 10 ITICK=1, 10
          XPLT = FLOAT(ITICK)/10.0
          CALL PLOT(XMOD(XPLT),YMOD(0.003),3)
          CALL PLOT(XMOD(XPLT),YMOD(-.003),2)
   10   CONTINUE
C
        XL0 = XMOD(XBMIN) + 1.0*CH
        YL0 = YMOD(YBMAX) + 3.0*CH
      ENDIF
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWPEN(2)
      CALL PLTAIR(X,XP,Y,YP,S,N, XOFF,XSF,YOFF,YSF,'black')
C
      XLAB = XL0
      YLAB = YL0
      CHL = CH
      DO 40 K = NPPAI, 1, -1
        IP = IPPAI(K)
        IF(IP.EQ.0) GO TO 40
C
C------- plot airfoil if it's archived
         NXY = NXYPOL(IP)
         IF(NXY.GT.1) THEN
          CALL SCALC(CPOLXY(1,1,IP),CPOLXY(1,2,IP),W3,NXY)
          CALL SPLINE(CPOLXY(1,1,IP),W1,W3,NXY)
          CALL SPLINE(CPOLXY(1,2,IP),W2,W3,NXY)
C
          CALL NEWCOLOR(ICOLP(IP))
          CALL PLTAIR(CPOLXY(1,1,IP),W1,
     &                CPOLXY(1,2,IP),W2, W3,NXY,
     &                XOFF,XSF,YOFF,YSF,' ')
C
C-------- also plot its number and name
          CALL STRIP(NAMEPOL(IP),NNAMEP)
          PFLT = FLOAT(IP)
          CALL PLNUMB(XLAB,YLAB,CHL,PFLT,0.0,-1)
          CALL PLCHAR(XLAB+3.0*CHL,YLAB,CHL,NAMEPOL(IP),0.0,NNAMEP)
          YLAB = YLAB + 2.5*CHL
         ENDIF
 40   CONTINUE
C
      CALL PLFLUSH
C
      RETURN
      END ! PPAPLT


 
      SUBROUTINE RESETSCL
C---- Resets scales, offsets for zooming 
C     uses  offsets XOFF,YOFF 
C     scale factors XSF,YSF
      INCLUDE 'XFOIL.INC'
      XOFF = 0.0
      YOFF = 0.0
      XSF  = 1.0
      YSF  = 1.0
      RETURN
      END
 





