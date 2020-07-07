***********************************************************************
C    Module:  xtcam.f
C 
C    Copyright (C) 2000 Harold Youngren, Mark Drela
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

      SUBROUTINE CAMB
C-------------------------------------------
C     Camber modification routine.
C-------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      CHARACTER*72 LINE
      CHARACTER*4 COMAND, COMOLD
      CHARACTER*128 COMARG, ARGOLD
      CHARACTER*1 ANS
C
      REAL XBOX(2), YBOX(2)
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR, LRECALC, LCLEAR, LGPARSAVE
C
      EXTERNAL NEWPLOTC
C
      DATA LMASK0, LMASK1, LMASK2, LMASK3 / -1, -32640, -30584, -21846 /
C
 1000 FORMAT(A)
C
      LGPARSAVE = LGPARM
      COMAND = '****'
      COMARG = ' '
      LRECALC = .FALSE.
      LCLEAR  = .TRUE.
      LU = 8
C
      COMOLD = COMAND
      ARGOLD = COMARG
C
      IF(.NOT.LPLCAM) THEN
       WRITE(*,*) 'Enabling camber,thickness plotting'
       LPLCAM = .TRUE.
       CALL GOFINI
      ENDIF
C
C--- Check chordline direction (should be unrotated for camber routines
C    to function correctly
      XLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XTE = 0.5*(XB(1)+XB(NB))
      YTE = 0.5*(YB(1)+YB(NB))
      AROT = ATAN2(YLE-YTE,XTE-XLE) / DTOR
      IF(ABS(AROT).GT.1.0) THEN
        WRITE(*,*) ' '
        WRITE(*,*) 'Warning: CAMB does not work well on rotated foils'
        WRITE(*,*) 'Current chordline angle: ',AROT
        WRITE(*,*) 'Proceeding anyway...'
      ENDIF
C
      CHS = 0.5*CHG
      LDCPLOT = .FALSE.
      LGPARM = .NOT.LDCPLOT
C
      WRITE(*,1200)
C
C--------------------------------------------------------------
C---- pick up here to initialize camber and loading
 100  CONTINUE
C
C---- find leftmost point
cc    CALL LEFIND(SBL,XB,XBP,YB,YBP,SB,NB)
      CALL XLFIND(SBL,XB,XBP,YB,YBP,SB,NB)
C
      XBL = SEVAL(SBL, XB,XBP,SB,NB)
      YBL = SEVAL(SBL, YB,YBP,SB,NB)
      XBR = 0.5*(XB(1)+XB(NB))
      YBR = 0.5*(YB(1)+YB(NB))
C
C---- set "chordline" axis vector for camber,thickness definitions
      XBCH = XBR - XBL
cc    YBCH = YBR - YBL
      YBCH = 0.
      SBCH = SQRT(XBCH**2 + YBCH**2)
C
C---- find the current buffer airfoil camber and thickness
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
C
ccc      write(*,*) 'xc0 xc1', xcm(1) , xcm(ncm)
      NCAM = MIN( 201 , NTX )
      DO K=1, NCAM
        XCAM(K) = XCM(1) + (XCM(NCM)-XCM(1))*FLOAT(K-1)/FLOAT(NCAM-1)
      ENDDO
C
      IF(LCLEAR) THEN
C---- initialize added camber to zero
       NCADD = 2
       XCADD(1) = XCAM(1)
       XCADD(2) = XCAM(NCAM)
       YCADD(1) = 0.0
       YCADD(2) = 0.0
C---- initialize added loading to zero
       NPADD = 2
       XPADD(1) = XCAM(1)
       XPADD(2) = XCAM(NCAM)
       YPADD(1) = 0.0
       YPADD(2) = 0.0
C---- spline added camber line y(x) and added loading dCp(x)
       CALL SEGSPL(YCADD,YCADDP,XCADD,NCADD)
       CALL SEGSPL(YPADD,YPADDP,XPADD,NPADD)
C----- interpolate to dense plotting array
       DO K=1, NCAM
         YCAM(K)  = SEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
         YCAMP(K) = DEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
         PCAM(K)  = SEVAL(XCAM(K),YPADD,YPADDP,XPADD,NPADD)
         PCAMP(K) = DEVAL(XCAM(K),YPADD,YPADDP,XPADD,NPADD)
       ENDDO
       LCLEAR = .FALSE.
      ENDIF
C
C--------------------------------------------------------------
C---- pick up here to find and display current camber and added camber line properties
 200  CONTINUE
C
      WRITE(*,*) 
      WRITE(*,*) 'Buffer airfoil thickness and camber:'
      CALL TCBUF
C
      XMX = 0.0
      YMX = 0.0
      DO K=1, NCAM
        IF(ABS(YCAM(K)) .GT. ABS(YMX)) THEN
         XMX = XCAM(K)
         YMX = YCAM(K)
        ENDIF
      END DO
      CHRD = XCAM(NCAM) - XCAM(1)
      ALE = ATAN( DEVAL(XCAM(1)   ,YCAM,YCAMP,XCAM,NCAM) ) / DTOR
      ATE = ATAN( DEVAL(XCAM(NCAM),YCAM,YCAMP,XCAM,NCAM) ) / DTOR
      WRITE(*,1100) ALE, ATE, YMX/CHRD, XMX/CHRD
 1100 FORMAT(/' Added camber line incidence at LE = ', F6.2, '  deg.',
     &       /' Added camber line incidence at TE = ', F6.2, '  deg.',
     &       /' Max added camber y/c = ', F8.4, '  at x/c = ', F7.3  )
C
C--------------------------------------------------------------
C---- pick up here to replot everything
 300  CONTINUE
      LGPARM = .NOT.LDCPLOT
      CALL PLTINI
      CALL PLOTG
      CALL PLOTC
C
C==================================================
C---- top of menu loop 
  500 CALL ASKC('..CAMB^',COMAND,COMARG)
C
C---- process previous command ?
      IF(COMAND(1:1).EQ.'!') THEN
        IF(COMOLD.EQ.'****') THEN
          WRITE(*,*) 'Previous ..CAMB command not valid'
          GO TO 500
        ELSE
          COMAND = COMOLD
          COMARG = ARGOLD
        ENDIF
      ENDIF
C
      IF(COMAND.EQ.'    ') THEN
C----- just <return> was typed... clean up plotting and exit CAMP
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL CLRZOOM
       LGPARM = LGPARSAVE
       RETURN
      ENDIF
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
      IF(COMAND.EQ.'    ') THEN
       IF(LPLOT) CALL PLEND
       RETURN
C
      ELSEIF(COMAND.EQ.'?   ') THEN
       WRITE(*,1200)
 1200  FORMAT(
     &  /'   <cr>    Return to GDES'
     &  /'   TFAC rr Scale existing thickness and camber'
     &  /'   TSET rr Set new thickness and camber'
     &  /'   HIGH rr Move camber and thickness highpoints'
     &  /'   WRTT f  Write airfoil thickness x/c,t/c to file'
     &  /'   WRTC f  Write airfoil camber    x/c,y/c to file'
     & //'   RDAC    Read   added camber  x/c,y/c from file'
     &  /'   SETC    Set    added camber  x/c,y/c from camberline'
     &  /'   INPC    Input  added camber  x/c,y/c from keyboard'
     &  /'   MODC    Modify added camber  x/c,y/c with cursor'
     &  /'   INPP    Input  added loading x/c,DCp from keyboard'
     &  /'   MODP    Modify added loading x/c,DCp with cursor'
     &  /'   SLOP    Toggle modified-camber,dCp slope matching flag'
     &  /'   SCAL r  Scale the added camber'
     &  /'   CLR     Clear the added camber'
     &  /'   ADD     Add added camber to the existing camberline'
     & //'   DCPL    Toggle DCp plot'
     &  /'   CPLI rr Change DCp axis plot limits'
     & //'   Blow    Blowup plot region'
     &  /'   Rese    Reset plot scale and origin'
     & //'   SIZE r   Change absolute plot-object size'
     &  /'  .ANNO     Annotate plot'
     &  /'   HARD     Hardcopy current plot')
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
      ELSEIF(COMAND.EQ.'TFAC') THEN
       CALL TCSCAL(RINPUT,NINPUT)
       GO TO 100
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TSET') THEN
       CALL TCSET(RINPUT,NINPUT)
       GO TO 100
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'HIGH') THEN
       CALL HIPNT(RINPUT,NINPUT)
       GO TO 100
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'WRTT') THEN
       IF(COMARG.EQ.' ') THEN
        CALL ASKS('Enter output thickness filename^',FNAME)
       ELSE
        FNAME = COMARG
       ENDIF
C
       OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=12)
       WRITE(*,*)
       WRITE(*,*) 'Output file exists.  Overwrite?  Y'
       READ(*,1000) ANS
       IF(INDEX('Nn',ANS).EQ.0) GO TO 13
C
       CLOSE(LU)
       WRITE(*,*) 'Current thickness not saved.'
       GO TO 500
C
 12    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=15)
 13    REWIND(LU)
C
C--- Write out normalized camber coordinates (x/c in range 0->1, y/c)
       WRITE(LU,1000) 'Thickness: '//NAME
       DO K = 1, NTK
         XTN = (XTK(K)-XTK(1))/XBCH
         YTN = (YTK(K)-YTK(1))/XBCH
         KM = MAX( 1 , K-1 )
         KP = MIN( NTK , K+1 )
         DCDX = (YTK(KP)-YTK(KM)) / (XTK(KP)-XTK(KM))
         WRITE(LU,14) XTN, YTN, DCDX
       END DO
       CLOSE(LU)
       GO TO 500
C
 14    FORMAT(1X, 3G15.7)
C
 15    WRITE(*,*) 'Error opening thickness save file'
       GO TO 500
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'WRTC') THEN
       IF(COMARG.EQ.' ') THEN
        CALL ASKS('Enter output camber filename^',FNAME)
       ELSE
        FNAME = COMARG
       ENDIF
C
       OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=22)
       WRITE(*,*)
       WRITE(*,*) 'Output file exists.  Overwrite?  Y'
       READ(*,1000) ANS
       IF(INDEX('Nn',ANS).EQ.0) GO TO 23
C
       CLOSE(LU)
       WRITE(*,*) 'Current camber not saved.'
       GO TO 500
C
 22    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=25)
 23    REWIND(LU)
C
C--- Write out normalized camber coordinates (x/c in range 0->1, y/c)
       WRITE(LU,1000) 'Camber: '//NAME
       DO K = 1, NCM
         WRITE(LU,24) (XCM(K)-XCM(1))/XBCH,(YCM(K)-YCM(1))/XBCH
       END DO
       CLOSE(LU)
       GO TO 500
C
 24    FORMAT(1X, 3G15.7)
C
 25    WRITE(*,*) 'Error opening camber save file'
       GO TO 500
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'RDAC ') THEN
       CALL ASKS('Enter added camber filename^',FNAME)
       OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=31)
       READ(LU,1000,ERR=30,END=30) LINE
       NCADD = 0
       DO K = 1, NTX
         READ(LU,*,ERR=30,END=30) XX,YY
         NCADD = NCADD + 1
         XCADD(NCADD) = XX
         YCADD(NCADD) = YY
       END DO
 30    CLOSE(LU)
       IF(NCADD.LE.1 .OR.  (XCADD(NCADD)-XCADD(1)).EQ.0.0) THEN
         NCADD = 2
         XCADD(1) = XCAM(1)
         XCADD(2) = XCAM(NCAM)
         YCADD(1) = 0.0
         YCADD(2) = 0.0
         WRITE(*,*) 'No added camber points found'
         GO TO 100
       ENDIF
C----- normalize input camber to x/c range 0->1
       XCORG = XCADD(1)
       XCSCL = XCADD(NCADD) - XCORG
       DO K=1, NCADD
         XCADD(K) = (XCADD(K)-XCORG) / XCSCL
         YCADD(K) =  YCADD(K)        / XCSCL
       ENDDO
C----- reorigin and scale added camber to camber line coordinates
       DO K=1, NCADD
         XCADD(K) = XCAM(1) + XCADD(K)*XBCH - YCADD(K)*YBCH
         YCADD(K) =           XCADD(K)*YBCH + YCADD(K)*XBCH
       ENDDO
C----- spline camber line y(x)
       CALL SEGSPL(YCADD,YCADDP,XCADD,NCADD)
C----- interpolate to dense plotting array
       DO K=1, NCAM
         YCAM(K)  = SEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
         YCAMP(K) = DEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
       ENDDO
       LDCPLOT = .FALSE.
       GO TO 200

 31    WRITE(*,*)
       WRITE(*,*) 'Error opening added camber file'
       GO TO 500
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SETC') THEN
C----- Set added camber from camberline
       NCADD = NCM
       DO K=1, NCM
         XCADD(K) = XCM(K)
         YCADD(K) = YCM(K)
       END DO 
C----- spline added camber line y(x)
       CALL SEGSPL(YCADD,YCADDP,XCADD,NCADD)
C
C----- interpolate to dense plotting array
       DO K=1, NCAM
         YCAM(K)  = SEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
         YCAMP(K) = DEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
       ENDDO
       LDCPLOT = .FALSE.
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'INPC') THEN
C----- Manual input of camber points
 40    WRITE(*,2000)
 2000  FORMAT(/' Manual input of camber x/c,y/c:',
     &       //' Input x/c, y/c pairs from  x/c = 0  to  x/c = 1',
cc     &       /' Identical successive points enable a slope break',
     &        /' <cr> ends input')
C
C--- Points of x/c, y/c are added to existing definition of added camber line
       CALL GETCOLOR(ICOL0)
       CALL NEWCOLORNAME('RED')
       NCADD = 0
       DO 45 I=1, 2*IQX
 43      READ(*,1000,ERR=44) LINE
         IF(LINE.EQ.' ') GO TO 46
         READ(LINE,*,ERR=44,END=44) XX,YY
         IF(XX.LE.0.0) THEN
          XX = 0.0
         ELSEIF(XX.GE.1.0) THEN
          XX = 1.0
         ENDIF
         NCADD = NCADD + 1
         XCADD(NCADD) = XCAM(1) + XX*XBCH - YY*YBCH
         YCADD(NCADD) =           XX*YBCH + YY*XBCH
C
         XPL = XSF*(XCADD(NCADD)-XOFF)
         YPL = YSF*(YCADD(NCADD)-YOFF-DYOFFC)
         CALL PLSYMB(XPL,YPL,CHS*XSF,1,0.0,I-1)
         CALL PLFLUSH
         GO TO 45
 44      WRITE(*,*) 'try again'
         GO TO 43
 45    CONTINUE
C----- Sort points allowing duplicates for slope breaks 
 46    CALL SORTDUP(NCADD,XCADD,YCADD)
       CALL FIXDUP (NCADD,XCADD,YCADD)
       CALL NEWCOLOR(ICOL0)
C----- spline camber line y(x)
       CALL SEGSPL(YCADD,YCADDP,XCADD,NCADD)
C
C----- interpolate to dense plotting array
       DO K=1, NCAM
         YCAM(K)  = SEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
         YCAMP(K) = DEVAL(XCAM(K),YCADD,YCADDP,XCADD,NCADD)
       ENDDO
       LDCPLOT = .FALSE.
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'INPP') THEN
C----- Manual input of loading points
 50    WRITE(*,3000)
 3000  FORMAT(/' Manual input of loading x/c, DCp:',
     &       //' Input x/c, DCp pairs from  x/c = 0  to  x/c = 1',
cc     &       /' Identical successive points enable a slope break',
     &        /' <cr> ends input')
C
       CALL GETPEN(IPN)
       CALL GETCOLOR(ICOL0)
C
       CALL NEWPEN(1)
       CHL = 1.5*CHG
       YOFFP = (DYOFFP+YOFF)/YSFP
       CALL GRDAIR(XPMIN,XPMAX,YPMIN,YPMAX,DXYG,DXYP,CHG,.FALSE.,.TRUE.,
     &             XOFF,XSF,YOFFP,YSF*YSFP, LMASK2)
       CALL NEWCOLORNAME('RED')
       CALL NEWPEN(2)
       XLAB = (XPMIN         -XOFF )*XSF      - 4.0*CHL
       YLAB = (YPMAX-0.5*DXYP-YOFFP)*YSF*YSFP - 0.6*CHL
       CALL PLCHAR(XLAB,YLAB,CHL,' Cp',0.0,3)
       CALL PLMATH(XLAB,YLAB,CHL,'O  ',0.0,3)
       CALL PLFLUSH
C
C--- Points of x/c, dCp are added to existing definition of loading line
       DO 55 I=1, 2*IQX
 53      READ(*,1000,ERR=54) LINE
         IF(LINE.EQ.' ') GO TO 56
         READ(LINE,*,ERR=54) XX,YY
         IF(XX.LE.0.0) THEN
           XX = 0.0
          ELSEIF(XX.GE.1.0) THEN
           XX = 1.0
         ENDIF
         NPADD = NPADD + 1
         XPADD(NPADD) = XCAM(1) + XX*XBCH
         YPADD(NPADD) = YY
C
cc       YOFFP = (DYOFFP*YOFF)/YSFP
         XPL = (XPADD(NPADD)-XOFF )*XSF
         YPL = (YPADD(NPADD)-YOFFP)*YSF*YSFP
         CALL PLSYMB(XPL,YPL,CHS,1,0.0,I-1)
         CALL PLFLUSH
         GO TO 55
 54      WRITE(*,*) 'try again'
         GO TO 53
 55    CONTINUE
C----- Sort points allowing duplicates for slope breaks 
 56    CONTINUE
       CALL SORTDUP(NPADD,XPADD,YPADD)
       CALL FIXDUP (NPADD,XPADD,YPADD)
C
       CALL NEWCOLOR(ICOL0)
       CALL NEWPEN(IPN)
C
C----- spline loading DCp(x)
       CALL SEGSPL(YPADD,YPADDP,XPADD,NPADD)
C
C----- interpolate to dense plotting array
       DO K=1, NCAM
         PCAM(K)  = SEVAL(XCAM(K),YPADD,YPADDP,XPADD,NPADD)
         PCAMP(K) = DEVAL(XCAM(K),YPADD,YPADDP,XPADD,NPADD)
       ENDDO
C
C----- calculate camber line corresponding to specified loading
       CALL CPCAM(NCAM,XCAM,YCAM,YCAMP,PCAM,PCAMP)
C
C----- calculate added lift and moment from added loading
       CLX = 0.0
       CMX = 0.0
       DO K=1, NCAM-1
        DX =      XCAM(K+1) - XCAM(K)
        XA = 0.5*(XCAM(K+1) + XCAM(K))
        PA = 0.5*(PCAM(K+1) + PCAM(K))
        CLX = CLX + PA*DX
        CMX = CMX + PA*DX*(XCMREF-XA)
       END DO  
       WRITE(*,1110) CLX, CMX
C
       LDCPLOT = .TRUE.
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'MODC') THEN
C----- Interactively modify camber
       XBOX(1) = XMARG
       XBOX(2) = XPAGE-XMARG
       YBOX(1) = YMARG
       YBOX(2) = YPAGE-YMARG
       XOFF1 = XOFF
       YOFF1 = YOFF+DYOFFC
       XSF1 = XSF
       YSF1 = YSF
       CALL MODIFY(NTX,1,NCAM,1,1,
     &             XCAM,YCAM,YCAMP, LCSLOP,
     &             K1,K2,ISMOD,IFMOD,
     &             XBOX,YBOX, XBOX,YBOX,SIZE,
     &             XOFF1,YOFF1,XSF1,YSF1, 'RED',' ',
     &             NEWPLOTC)
       LDCPLOT = .FALSE.
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'MODP') THEN
C----- Interactively modify loading
       IF(.NOT.LDCPLOT) THEN
        LDCPLOT = .TRUE.
        LGPARM = .NOT.LDCPLOT
        CALL PLTINI
        CALL PLOTG
        CALL PLOTC
       ENDIF
       XBOX(1) = XMARG
       XBOX(2) = XPAGE-XMARG
       YBOX(1) = YMARG
       YBOX(2) = YPAGE-YMARG
       XOFF1 = XOFF
       YOFF1 = (DYOFFP+YOFF)/YSFP
       XSF1 = XSF
       YSF1 = YSF*YSFP
       CALL MODIFY(NTX,1,NCAM,1,1,
     &             XCAM,PCAM,PCAMP, LCSLOP,
     &             K1,K2,ISMOD,IFMOD,
     &             XBOX,YBOX, XBOX,YBOX,SIZE,
     &             XOFF1,YOFF1,XSF1,YSF1, 'RED',' ',
     &             NEWPLOTC)
C
C----- calculate camber line corresponding to specified loading
       CALL CPCAM(NCAM,XCAM,YCAM,YCAMP,PCAM,PCAMP)
C
C----- calculate added lift and moment from added loading
       CLX = 0.0
       CMX = 0.0
       DO K=1, NCAM-1
        DX =      XCAM(K+1) - XCAM(K)
        XA = 0.5*(XCAM(K+1) + XCAM(K))
        PA = 0.5*(PCAM(K+1) + PCAM(K))
        CLX = CLX + PA*DX
        CMX = CMX + PA*DX*(XCMREF-XA)
       END DO  
       WRITE(*,1110) CLX, CMX
C
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CLR ') THEN
C----- Clear the added camber
       LCLEAR  = .TRUE.
       LDCPLOT = .FALSE.
       GO TO 100
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SCAL') THEN
C----- Scale camber
       IF(NINPUT.GE.1) THEN
        SCAL = RINPUT(1)
       ELSE
        SCAL = 1.0
        CALL ASKR('Enter camber scaling factor^',SCAL)
       ENDIF
C
C--- Scale added camber user arrays
       DO I = 1, NCADD
         YCADD(I)  = YCADD(I) *SCAL
         YPADD(I)  = YPADD(I) *SCAL
         YCADDP(I) = YCADDP(I)*SCAL
         YPADDP(I) = YPADDP(I)*SCAL
       END DO
C
C--- Scale added camber arrays
       DO I = 1, NCAM
         YCAM(I)   = YCAM(I) *SCAL
         YCAMP(I)  = YCAMP(I)*SCAL
         PCAM(I)   = PCAM(I) *SCAL
         PCAMP(I)  = PCAMP(I)*SCAL
       END DO
C
C----- calculate added lift and moment from added loading
       CLX = 0.0
       CMX = 0.0
       DO K=1, NCAM-1
        DX =      XCAM(K+1) - XCAM(K)
        XA = 0.5*(XCAM(K+1) + XCAM(K))
        PA = 0.5*(PCAM(K+1) + PCAM(K))
        CLX = CLX + PA*DX
        CMX = CMX + PA*DX*(XCMREF-XA)
       END DO  
       IF(CLX.NE.0.0 .AND. CMX.NE.0.0) WRITE(*,1110) CLX, CMX
C
       COMOLD = COMAND
       ARGOLD = COMARG
C
C
C----- go replot new shape and resume menu loop
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'ADD ') THEN
C----- Add camber to camberline
       CALL SEGSPL(YCAM,YCAMP,XCAM,NCAM)
C
C----- go over each point, changing the camber line appropriately
       DO I=1, NB
C------- coordinates of point on the opposite side with the same chord x value
         CALL SOPPS(SBOPP, SB(I),XB,XBP,YB,YBP,SB,NB,SBL)
         XBOPP = SEVAL(SBOPP,XB,XBP,SB,NB)
         YBOPP = SEVAL(SBOPP,YB,YBP,SB,NB)
C
C------- set present camber height
         OLDCAM = 0.5*(YB(I)+YBOPP)*XBCH/SBCH
     &          - 0.5*(XB(I)+XBOPP)*YBCH/SBCH
C
C------- add on new camber
         CAM = OLDCAM
     &       + SEVAL(XB(I),YCAM,YCAMP,XCAM,NCAM)
C
C------- set new y coordinate by changing camber & thickness appropriately
         W1(I) = CAM  +  0.5*(YB(I)-YBOPP)
       END DO
C
       DO I=1, NB
         YB(I) = W1(I)
       END DO
C
       CALL SCALC(XB,YB,SB,NB)
       CALL SEGSPL(XB,XBP,SB,NB)
       CALL SEGSPL(YB,YBP,SB,NB)
C
       CALL GEOPAR(XB,XBP,YB,YBP,SB,NB,W1,
     &             SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &             EI11BA,EI22BA,APX1BA,APX2BA,
     &             EI11BT,EI22BT,APX1BT,APX2BT,
     &             THICKB,CAMBRB )
C
       LDCPLOT = .FALSE.
C---- reinitialize added camber to zero
       LCLEAR  = .TRUE.
       GO TO 100
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SLOP') THEN
       LCSLOP = .NOT.LCSLOP
       IF(LCSLOP) THEN
        WRITE(*,*) 'Modified segment will be',
     &             ' made tangent at endpoints'
       ELSE
        WRITE(*,*) 'Modified segment will not be',
     &             ' made tangent at endpoints'
       ENDIF
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'BLOW' .OR.
     &       COMAND.EQ.'B   ') THEN
       XWS = XWIND/SIZE
       YWS = YWIND/SIZE
       CALL OFFGET(XOFF,YOFF,XSF,YSF,XWS,YWS, .TRUE. , .TRUE. )
       SF = MIN(XSF,YSF)
       XSF = SF
       YSF = SF
       GO TO 300
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'RESE' .OR.
     &       COMAND.EQ.'R   ') THEN
       CALL PLTINI
       CALL GOFINI
       CALL PLOTG
cc       CALL RESETSCL
       GO TO 300
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'DCPL') THEN
C----- Toggle DCp plot flag
       LDCPLOT = .NOT.LDCPLOT
       GO TO 200
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CPLI') THEN
       IF    (NINPUT.GE.2) THEN
        YPMN = RINPUT(1)
        YPMX = RINPUT(2)
       ELSEIF(NINPUT.GE.1) THEN
        YPMIN = RINPUT(1)
        CALL ASKR('Enter max DCp^',YPMX)
       ELSE
        CALL ASKR('Enter min DCp^',YPMN)
        CALL ASKR('Enter max DCp^',YPMX)
       ENDIF
       IF(YPMX-YPMN.GT.0.0) THEN
        CALL AXISADJ(YPMN,YPMX,PSPAN,DXYP,NTICS)
        YPMIN = YPMN
        YPMAX = YPMX
        CALL GOFINI
       ENDIF
       GO TO 300
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SIZE') THEN
       IF(NINPUT.GE.1) THEN
        SIZE = RINPUT(1)
       ELSE
        WRITE(*,*) 'Current plot-object size =', SIZE
        CALL ASKR('Enter new plot-object size^',SIZE)
       ENDIF
       CALL GOFINI
       GO TO 300
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'ANNO') THEN
       IF(LPLOT) THEN
        CALL ANNOT(CH)
       ELSE
        WRITE(*,*) 'No active plot to annotate'
       ENDIF
       GO TO 300
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'HARD') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVRP)
       GO TO 300
C
C-------------------------------------------------------
      ELSE
       WRITE(*,8000) COMAND
 8000  FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
C
      ENDIF
      GO TO 500
C
 1110 FORMAT(/' Delta Cp loading gives delta CL = ',F7.3,
     &       /'                        delta CM = ',F7.3)
C
      END ! CAMB



      SUBROUTINE NEWPLOTC
      CALL PLTINI
      CALL PLOTG
      CALL PLOTC
      RETURN
      END



      SUBROUTINE PLOTC
C------------------------------------------------------
C     Plots camber, thickness on its own axis above airfoil plot
C     Also plots deltaCP distribution above the camber,thickness 
C     on its own axis above airfoil plot if LDCPLOT is set
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      DATA LMASK0, LMASK1, LMASK2, LMASK3 / -1, -32640, -30584, -21846 /
C
      CALL GETCOLOR(ICOL0)
      CALL GETPAT(IPAT0)
C
      CHS = 0.6*CHG
C
      IF(LDCPLOT) THEN
C----- current DCp loading is valid... plot it
       CALL NEWPEN(1)
       CHL = 1.5*CHG
       YOFFP = (DYOFFP+YOFF)/YSFP
       CALL GRDAIR(XGMIN,XGMAX,YPMIN,YPMAX,DXYG,DXYP,CHG,.FALSE.,.TRUE.,
     &             XOFF,XSF,YOFFP,YSFP*YSF, LMASK2)
C
       CALL NEWCOLORNAME('RED')
       CALL NEWPEN(2)
       XLAB = (XPMIN         -XOFF )*XSF      - 4.0*CHL
       YLAB = (YPMAX-0.5*DXYP-YOFFP)*YSFP*YSF - 0.6*CHL
       CALL PLCHAR(XLAB,YLAB,CHL,' Cp',0.0,3)
       CALL PLMATH(XLAB,YLAB,CHL,'O  ',0.0,3)
C
       CALL XYLINE(NCAM,XCAM,PCAM,XOFF,XSF,YOFFP,YSFP*YSF,1)
ccc       CALL XYSYMB(NCAM,XCAM,PCAM,XOFF,XSF,YOFFP,YSFP*YSF,CHS,1)
       CALL NEWCOLOR(ICOL0)
C
cC----- plot derived camber line in dotted line
c       CALL NEWPAT(LMASK3)
c      ELSE
cC----- plot specified camber line in solid line
c       CALL NEWPAT(LMASK0)
      ENDIF
C
      CALL NEWPEN(1)
      CALL NEWCOLORNAME('RED')
      CALL XYLINE(NCAM,XCAM,YCAM,XOFF,XSF,YOFF+DYOFFC,YSF,1)
ccc      CALL XYSYMB(NCAM,XCAM,YCAM,XOFF,XSF,YOFF+DYOFFC,YSF,CHS,1)
C
      CALL NEWCOLOR(ICOL0)
      CALL NEWPAT(IPAT0)
      CALL PLFLUSH
C
      RETURN
      END ! PLOTC




      SUBROUTINE ZERCAM
C-----------------------------------------
C     Zeros out camber of buffer airfoil
C-----------------------------------------
      INCLUDE 'XFOIL.INC'
C
      WRITE(*,*) 'Setting current camber to zero.'
      TFAC = 1.0
      CFAC = 0.0
      CALL THKCAM(TFAC,CFAC)
C
C---- make points exact mirror images
      CALL YSYM(XB,XBP,YB,YBP,SB,2*IQX,NB,1,W1,W2)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB,W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      CALL PLNEWP('magenta')
C
      LGEOPL = .FALSE.
C
      RETURN
      END ! ZERCAM



      SUBROUTINE TCBUF
C------------------------------------------------------
C     Reports buffer airfoil thickness and camber
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C--- find the current buffer airfoil camber and thickness
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
      CALL GETMAX(XCM,YCM,YCMP,NCM,CXMAX,CYMAX)
      CALL GETMAX(XTK,YTK,YTKP,NTK,TXMAX,TYMAX)
C
      WRITE(*,1000) 2.0*TYMAX,TXMAX, CYMAX,CXMAX
 1000 FORMAT( ' Max thickness = ',F8.4,'  at x = ',F7.3,
     &       /' Max camber    = ',F8.4,'  at x = ',F7.3)
C
      RETURN
      END ! TCBUF


      SUBROUTINE TCSCAL(RINPUT,NINPUT)
      DIMENSION RINPUT(*)
C------------------------------------------------------
C     Finds buffer airfoil thickness and/or camber,
C     plots thickness, camber and airfoil, 
C     and scales t and/or c by user input factors
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C--- find the current buffer airfoil camber and thickness
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
      CALL GETMAX(XCM,YCM,YCMP,NCM,CXMAX,CYMAX)
      CALL GETMAX(XTK,YTK,YTKP,NTK,TXMAX,TYMAX)
C
      WRITE(*,1000) 2.0*TYMAX,TXMAX, CYMAX,CXMAX
C
      IF    (NINPUT .GE. 2) THEN
        TFAC = RINPUT(1)
        CFAC = RINPUT(2)
      ELSEIF(NINPUT .GE. 1) THEN
        TFAC = RINPUT(1)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CFAC = 1.0
         CALL ASKR('Enter new/old camber    scale factor^',CFAC)
        ENDIF
      ELSE
        TFAC = 1.0
        CALL ASKR( 'Enter new/old thickness scale factor^',TFAC)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CFAC = 1.0
         CALL ASKR('Enter new/old camber    scale factor^',CFAC)
        ENDIF
      ENDIF
C
ccc      IF (TFAC.LT.0.0) TFAC = 0.0
      CALL THKCAM(TFAC,CFAC)
C
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
cc      IPLT = 1
cc      CALL PLOTC
C
 1000 FORMAT(/' Max thickness = ',F8.4,'  at x = ',F7.3,
     &       /' Max camber    = ',F8.4,'  at x = ',F7.3/)
C
      RETURN
      END ! TCSCAL


      SUBROUTINE TCSET(RINPUT,NINPUT)
      DIMENSION RINPUT(*)
C------------------------------------------------------
C     Finds buffer airfoil thickness and/or camber,
C     plots thickness, camber and airfoil, 
C     and scales t and/or c by user input factors
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C--- find the current buffer airfoil camber and thickness
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
      CALL GETMAX(XCM,YCM,YCMP,NCM,CXMAX,CYMAX)
      CALL GETMAX(XTK,YTK,YTKP,NTK,TXMAX,TYMAX)
C
      WRITE(*,1000) 2.0*TYMAX,TXMAX, CYMAX,CXMAX
 1000 FORMAT(/' Max thickness = ',F8.4,'  at x = ',F7.3,
     &       /' Max camber    = ',F8.4,'  at x = ',F7.3/)
C
cc      IPLT = 0
cc      CALL PLOTC
C
      IF    (NINPUT .GE. 2) THEN
        TNEW = RINPUT(1)
        CNEW = RINPUT(2)
      ELSEIF(NINPUT .GE. 1) THEN
        TNEW = RINPUT(1)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CNEW = 999
         CALL ASKR('Enter new max  camber   <ret> to skip^',CNEW)
        ENDIF
      ELSE
        TNEW = 999
        CALL  ASKR('Enter new max thickness <ret> to skip^',TNEW)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CNEW = 999
         CALL ASKR('Enter new max  camber   <ret> to skip^',CNEW)
        ENDIF
      ENDIF
C
      CFAC = 1.0
      TFAC = 1.0
      IF(CYMAX.NE.0.0 .AND. CNEW.NE.999.0) CFAC = CNEW / (    CYMAX)
      IF(TYMAX.NE.0.0 .AND. TNEW.NE.999.0) TFAC = TNEW / (2.0*TYMAX)
C
C---- sanity checks on scaling factors
      IF(ABS(TFAC) .GT. 100.0 .OR. ABS(CFAC) .GT. 100.0) THEN
        WRITE(*,1100) TFAC, CFAC
 1100   FORMAT(/' Questionable input...'
     &         /' Implied scaling factors are:', F13.2,' x thickness'
     &         /'                             ', F13.2,' x camber   ')
        CALL ASKL('Apply scaling factors?^',OK)
        IF(.NOT.OK) THEN
          WRITE(*,*) 'No action taken'
          RETURN
        ENDIF
      ENDIF
C
ccc      IF (TFAC.LT.0.0) TFAC = 0.0
      CALL THKCAM(TFAC,CFAC)
C
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
cc      IPLT = 1
cc      CALL PLOTC
C
      RETURN
      END ! TCSET



      SUBROUTINE THKCAM(TFAC,CFAC)
C---------------------------------------------------
C     Changes buffer airfoil thickness and camber
C---------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
C
C---This fails miserably with sharp LE foils, tsk,tsk,tsk HHY 4/24/01
C---- set baseline vector normal to surface at LE point
c      DXC = -DEVAL(SBLE,YB,YBP,SB,NB)
c      DYC =  DEVAL(SBLE,XB,XBP,SB,NB)
c      DSC = SQRT(DXC**2 + DYC**2)
c      DXC = DXC/DSC
c      DYC = DYC/DSC
C
C---Rational alternative 4/24/01 HHY
      XLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XTE = 0.5*(XB(1)+XB(NB))
      YTE = 0.5*(YB(1)+YB(NB))
      CHORD = SQRT((XTE-XLE)**2 + (YTE-YLE)**2)
C---- set unit chord-line vector
      DXC = (XTE-XLE) / CHORD
      DYC = (YTE-YLE) / CHORD
C
C---- go over each point, changing the y-thickness appropriately
      DO I=1, NB
C------ coordinates of point on the opposite side with the same x value
        CALL SOPPS(SBOPP, SB(I),XB,XBP,YB,YBP,SB,NB,SBLE)
        XBOPP = SEVAL(SBOPP,XB,XBP,SB,NB)
        YBOPP = SEVAL(SBOPP,YB,YBP,SB,NB)
C
C------ set new y coordinate by changing camber & thickness appropriately
        XCAVG =        ( 0.5*(XB(I)+XBOPP)*DXC + 0.5*(YB(I)+YBOPP)*DYC )
        YCAVG = CFAC * ( 0.5*(YB(I)+YBOPP)*DXC - 0.5*(XB(I)+XBOPP)*DYC )

        XCDEL =        ( 0.5*(XB(I)-XBOPP)*DXC + 0.5*(YB(I)-YBOPP)*DYC )
        YCDEL = TFAC * ( 0.5*(YB(I)-YBOPP)*DXC - 0.5*(XB(I)-XBOPP)*DYC )
C
        W1(I) = (XCAVG+XCDEL)*DXC - (YCAVG+YCDEL)*DYC
        W2(I) = (YCAVG+YCDEL)*DXC + (XCAVG+XCDEL)*DYC
      ENDDO
C
      DO I=1, NB
        XB(I) = W1(I)
        YB(I) = W2(I)
      ENDDO
      LGSAME = .FALSE.
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB,W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      RETURN
      END ! THKCAM



      SUBROUTINE HIPNT(RINPUT,NINPUT)
      DIMENSION RINPUT(*)
C------------------------------------------------------
C     Changes buffer airfoil 
C     thickness and/or camber highpoint
C------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      REAL XFN(5), YFN(5), YFNP(5), SFN(5)
C
C
C--- Check chordline direction (should be unrotated for camber routines)
C    to function correctly
      XLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XTE = 0.5*(XB(1)+XB(NB))
      YTE = 0.5*(YB(1)+YB(NB))
      AROT = ATAN2(YLE-YTE,XTE-XLE) / DTOR
      IF(ABS(AROT).GT.1.0) THEN
        WRITE(*,*) ' '
        WRITE(*,*) 'Warning: HIGH does not work well on rotated foils'
        WRITE(*,*) 'Current chordline angle: ',AROT
        WRITE(*,*) 'Proceeding anyway...'
      ENDIF
C
C
C---- find leftmost point location 
      CALL XLFIND(SBL,XB,XBP,YB,YBP,SB,NB)
      XBL = SEVAL(SBL,XB,XBP,SB,NB)
      YBL = SEVAL(SBL,YB,YBP,SB,NB)
C
 10   CONTINUE
C
C---- find the current buffer airfoil camber and thickness
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
C
C---- find the max thickness and camber
      CALL GETMAX(XCM,YCM,YCMP,NCM,CXMAX,CYMAX)
      CALL GETMAX(XTK,YTK,YTKP,NTK,TXMAX,TYMAX)
C
C
C---- make a picture and get some input specs for mods
cc      IPLT = 0
cc      CALL PLOTC
      WRITE(*,1010) 2.0*TYMAX,TXMAX, CYMAX,CXMAX
 1010 FORMAT(/' Max thickness = ',F8.4,'  at x = ',F7.3,
     &       /' Max camber    = ',F8.4,'  at x = ',F7.3/)
C
      IF    (NINPUT .GE. 2) THEN
        THPNT = RINPUT(1)
        CHPNT = RINPUT(2)
      ELSEIF(NINPUT .GE. 1) THEN
        THPNT = RINPUT(1)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CHPNT = 0.0
         CALL ASKR('Enter new camber    highpoint x: ^',CHPNT)
        ENDIF
      ELSE
        THPNT = 0.0
        CALL ASKR('Enter new thickness highpoint x: ^',THPNT)
        IF(LGSYM) THEN
         WRITE(*,*) 'Symmetry enforced:  Maintaining zero camber.'
        ELSE
         CHPNT = 0.0
         CALL ASKR('Enter new camber    highpoint x: ^',CHPNT)
        ENDIF
      ENDIF
C
      IF (THPNT.LE.0.0) THPNT = TXMAX
      IF (CHPNT.LE.0.0) CHPNT = CXMAX
C
C--- a simple cubic mapping function is used to map x/c to move highpoints
C
C    the assumption is that a smooth function (cubic, given by the old and 
C    new highpoint locations) maps the range 0-1 for x/c
C    into the range 0-1 for altered x/c distribution for the same y/c
C    thickness or camber (ie. slide the points smoothly along the x axis)
C
C--- shift thickness highpoint
      IF (THPNT .GT. 0.0) THEN
       XFN(1) = XTK(1)
       XFN(2) = TXMAX
       XFN(3) = XTK(NTK)
       YFN(1) = XTK(1)
       YFN(2) = THPNT
       YFN(3) = XTK(NTK)
       CALL SPLINA(YFN,YFNP,XFN,3)
       DO I = 1, NTK
         XTK(I) = SEVAL(XTK(I),YFN,YFNP,XFN,3)
       ENDDO
      ENDIF
C
C--- shift camber highpoint
      IF (CHPNT .GT. 0.0) THEN
       XFN(1) = XCM(1)
       XFN(2) = CXMAX
       XFN(3) = XCM(NCM)
       YFN(1) = XCM(1)
       YFN(2) = CHPNT
       YFN(3) = XCM(NCM)
       CALL SPLINA(YFN,YFNP,XFN,3)
       DO I = 1, NCM
         XCM(I) = SEVAL(XCM(I),YFN,YFNP,XFN,3)
       ENDDO
      ENDIF
C
cc      IPLT = 1
cc      CALL PLOTC
C
C      CALL ASKL('Is this acceptable? ^',OK)
C      IF(.NOT.OK) GO TO 10
C
C---- Make new airfoil from thickness and camber
C     new airfoil points are spaced to match the original
C--- HHY 4/24/01 got rid of splining vs X,Y vs S (buggy), now spline Y(X)
      CALL SEGSPL(YTK,YTKP,XTK,NTK)
      CALL SEGSPL(YCM,YCMP,XCM,NCM)
C
C
C---- for each orig. airfoil point setup new YB from camber and thickness
      DO 40 I=1, NB
C
C------ spline camber and thickness at original xb points
        YCC = SEVAL(XB(I),YCM,YCMP,XCM,NCM)
        YTT = SEVAL(XB(I),YTK,YTKP,XTK,NTK)
C
C------ set new y coordinate from new camber & thickness
        IF (SB(I) .LE. SBL) THEN
          YB(I) = YCC + YTT
         ELSE
          YB(I) = YCC - YTT
        ENDIF
C---- Add Y-offset for original leftmost (LE) point to camber
        YB(I) = YB(I) + YBL
   40 CONTINUE
      LGSAME = .FALSE.
C
      CALL SCALC(XB,YB,SB,NB)
      CALL SEGSPL(XB,XBP,SB,NB)
      CALL SEGSPL(YB,YBP,SB,NB)
C
      CALL GEOPAR(XB,XBP,YB,YBP,SB,NB,W1,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      RETURN
      END ! HIPNT


      SUBROUTINE GETCAM (XCM,YCM,NCM,XTK,YTK,NTK,
     &                   X,XP,Y,YP,S,N )
C------------------------------------------------------
C     Finds camber and thickness 
C     distribution for input airfoil 
C------------------------------------------------------
      REAL XCM(*), YCM(*)
      REAL XTK(*), YTK(*)
      REAL X(*),XP(*),Y(*),YP(*),S(*)
C
      CALL XLFIND(SL,X,XP,Y,YP,S,N)
      XL = SEVAL(SL,X,XP,S,N)
      YL = SEVAL(SL,Y,YP,S,N)
C
C---- go over each point, finding opposite points, getting camber and thickness
      DO 10 I=1, N
C------ coordinates of point on the opposite side with the same x value
        CALL SOPPS(SOPP, S(I), X,XP,Y,YP,S,N,SL)
        XOPP = SEVAL(SOPP,X,XP,S,N)
        YOPP = SEVAL(SOPP,Y,YP,S,N)

        IF(I.EQ.1) THEN
         XOPP = X(N)
         YOPP = Y(N)
        ELSEIF(I.EQ.N) THEN
         XOPP = X(1)
         YOPP = Y(1)
        ENDIF
C
C------ get camber and thickness
        XCM(I) = 0.5*(X(I)+XOPP)
        YCM(I) = 0.5*(Y(I)+YOPP)
        XTK(I) = 0.5*(X(I)+XOPP)
        YTK(I) = 0.5*(Y(I)-YOPP)
        YTK(I) = ABS(YTK(I))
c        if (XOPP.gt.0.9) then
c         write(*,*) 'cm i,x,y ',i,xcm(i),ycm(i)
c         write(*,*) 'tk i,x,y ',i,xtk(i),ytk(i)
c        endif
   10 CONTINUE
C
C---- Tolerance for nominally identical points
      TOL = 1.0E-5 * (S(N)-S(1))
ccc      TOL = 1.0E-3 * (S(N)-S(1))    ! Bad bug -- was losing x=1.0 point
C
C---- Sort the camber points
      NCM = N+1
      XCM(N+1) = XL
      YCM(N+1) = YL
      CALL SORTOL(TOL,NCM,XCM,YCM)
C
C--- Reorigin camber from LE so camberlines start at Y=0  4/24/01 HHY 
C    policy now to generate camber independent of Y-offsets 
      YOF = YCM(1)
      DO I = 1, NCM
        YCM(I) = YCM(I) - YOF
      END DO
C
C---- Sort the thickness points
      NTK = N+1
      XTK(N+1) = XL
      YTK(N+1) = 0.0
      CALL SORTOL(TOL,NTK,XTK,YTK)
C
      RETURN
      END ! GETCAM


      SUBROUTINE GETMAX(X,Y,YP,N,XMAX,YMAX)
      REAL X(*), Y(*), YP(*)
C------------------------------------------------
C     Calculates camber or thickness highpoint 
C     and x position
C------------------------------------------------
C
      XLEN = X(N) - X(1)
      XTOL = XLEN * 1.0E-5
C
      CALL SEGSPL(Y,YP,X,N)
C
C---- get approx max point and rough interval size
      YMAX0 = Y(1)
      XMAX0 = X(1)
      DO 5 I = 2, N
        IF (ABS(Y(I)).GT.ABS(YMAX0)) THEN
          YMAX0 = Y(I)
          XMAX0 = 0.5*(X(I-1) + X(I))
          DDX = 0.5*ABS(X(I+1) - X(I-1))
        ENDIF
 5    CONTINUE
      XMAX = XMAX0
C
C---- do a Newton loop to refine estimate
      DO 10 ITER=1, 10
        YMAX  = SEVAL(XMAX,Y,YP,X,N)
        RES   = DEVAL(XMAX,Y,YP,X,N)
        RESP  = D2VAL(XMAX,Y,YP,X,N)
        IF (ABS(XLEN*RESP) .LT. 1.0E-6) GO TO 20
          DX = -RES/RESP
          DX = SIGN( MIN(0.5*DDX,ABS(DX)) , DX)
          XMAX = XMAX + DX
          IF(ABS(DX) .LT. XTOL) GO TO 20
   10 CONTINUE
      WRITE(*,*)
     &  'GETMAX: Newton iteration for max camber/thickness failed.'
      YMAX = YMAX0
      XMAX = XMAX0
C
 20   RETURN
      END ! GETMAX



      SUBROUTINE CPCAM(N,X,Y,DYDX,P,DPDX)
      REAL X(*), Y(*), DYDX(*), P(*), DPDX(*)
C------------------------------------------------------------------
C     Generates y(x) camberline from specified DCp(x) distribution.
C
C     Input:  N       number of points
C             X(.)    x array
C             P(.)    DCp array
C             DPDX(.) dDCp/dx array
C
C     Output: Y(.)    y(x) array
C             DYDX(.) dy/dx array
C------------------------------------------------------------------
C---- 1 / 4 pi
      DATA QOPI / 7.9577471545948E-02 /
C
      P0 = P(1)
      P1 = P(N)
C
      X0 = X(1)
      X1 = X(N)
C         
C---- calculate Cauchy integral for y'(x) with removed singularity
      DO I=1, N

ccc     write(*,'(1x,i4,3f10.4)') i, x(i), p(i), dpdx(i)  !###@@@

        DYDX(I) = 0.0
        J = 1
        IF(I.EQ.J) THEN
         YP1 = DPDX(J)
        ELSE
         YP1 = (P(J) - P(I)) / (X(J) - X(I))
        ENDIF
        DO J=2, N
          IF(I.EQ.J) THEN
           YP2 = DPDX(J)
          ELSE
           YP2 = (P(J) - P(I)) / (X(J) - X(I))
          ENDIF
          DYDX(I) = DYDX(I) + 0.5*(YP1+YP2)*(X(J)-X(J-1))
          YP1 = YP2
        END DO
        DYDX(I) = QOPI*DYDX(I)
C
C------ add on removed part of Cauchy integral, further leaving out the
C-      possible infinities at LE and TE so that y(x) can be safely splined. 
C-      The infinities are analytically integrated, and added on to y(x)
C-      with the statement function YSING.
        IF(I.NE.1) THEN
         DYDX(I) = DYDX(I)
     &           - QOPI*(P(I) - P0)*LOG(X(I) - X0)
        ENDIF
        IF(I.NE.N) THEN
         DYDX(I) = DYDX(I)
     &           + QOPI*(P(I) - P1)*LOG(X1 - X(I))
        ENDIF
      END DO
C
C---- integrate regular part of y'(x) from LE
      Y(1) = 0.
      DO I=2, N
        Y(I) = Y(I-1)
     &       + 0.5*(DYDX(I) + DYDX(I-1))*(X(I) - X(I-1))
      END DO
C
C---- add on singular part of y(x) camber due to finite loadings P0,P1 at LE and TE
C-     dYSING/dX has logarithmic singularity at x=X0,X1
      DO I=1, N
        XT = X(I)
        YSING = QOPI*P1*((XT-X1)*LOG(MAX((X1-XT)/(X1-X0),1.E-6)) - XT)
     &        - QOPI*P0*((XT-X0)*LOG(MAX((XT-X0)/(X1-X0),1.E-6)) - XT)
        Y(I) = Y(I) + YSING
      ENDDO
C
C---- add offset and angle of attack to get y(0) = y(1) = 0
      Y0 = Y(1)
      Y1 = Y(N)
      DO I=1, N
        Y(I) = Y(I)
     &       - Y0*(X1  -X(I))/(X1-X0)
     &       - Y1*(X(I)-X0  )/(X1-X0)
      END DO
C
      RETURN
      END ! CPCAM
