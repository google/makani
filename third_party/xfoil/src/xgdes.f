C***********************************************************************
C    Module:  xgdes.f
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
      SUBROUTINE GDES
      INCLUDE 'XFOIL.INC'
      CHARACTER*4 COMAND, COMOLD
      LOGICAL LRECALC, LMODPL, LPLNEW
      DIMENSION XBOX(2), YBOX(2), XRF(2)
      DIMENSION XBT(IBX), YBT(IBX)
C
      CHARACTER*128 COMARG, ARGOLD
      CHARACTER*1 CHKEY
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
      EXTERNAL NEWPLOTG
C
      SAVE COMOLD, ARGOLD
C
      COMAND = '****'
      COMARG = ' '
      LRECALC = .FALSE.
C
      IF(NB.EQ.0) THEN
       WRITE(*,*)
       WRITE(*,*) '***  No airfoil available  ***'
       RETURN
      ENDIF
C
      LPLCAM = .FALSE.
      LSYM = .TRUE.
C
      WRITE(*,*)
      WRITE(*,*) 'You are working with the buffer airfoil'
C
      CALL PLTINI
      CALL GOFINI
      CALL PLOTG
C
C====================================================
C---- start of menu loop
 500  CONTINUE
      COMOLD = COMAND
      ARGOLD = COMARG
C
 501  IF(LGSYM) THEN
       CALL ASKC('.GDESs^',COMAND,COMARG)
      ELSE
       CALL ASKC('.GDES^',COMAND,COMARG)
      ENDIF
C
C--------------------------------------------------------
C---- process previous command ?
      IF(COMAND(1:1).EQ.'!') THEN
        IF(COMOLD.EQ.'****') THEN
          WRITE(*,*) 'Previous .GDES command not valid'
          GO TO 501
        ELSE
          COMAND = COMOLD
          COMARG  = ARGOLD
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
       LGSYM = .FALSE.
       LGEOPL = .FALSE.
       IF(.NOT.LGSAME) THEN
        WRITE(*,*)
        WRITE(*,*) 'Buffer airfoil is not identical to current airfoil'
       ENDIF
       CALL CLRZOOM
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
C--------------------------------------------------------
      IF(COMAND.EQ.'?   ') THEN
       WRITE(*,1050)
 1050  FORMAT(
     & /'   <cr>     Return to Top Level'
     & /'   !        Redo previous command'
     &//'   GSET     Set buffer  airfoil <== current airfoil'
     & /'   eXec     Set current airfoil <== buffer  airfoil'
     & /'   SYMM     Toggle y-symmetry flag'
     &//'   ADEG r   Rotate about origin (degrees)'
     & /'   ARAD r   Rotate about origin (radians)'
     & /'   Tran rr  Translate'
     & /'   Scal r   Scale about origin'
     & /'   LINS rr. Linearly-varying y scale'
     & /'   DERO     Derotate (set chord line level)'
     &//'   TGAP rr  Change trailing edge gap'
     & /'   LERA rr  Change leading edge radius'
     &//'   TCPL     Toggle thickness and camber plotting'
     & /'   TFAC rr  Scale existing thickness and camber'
     & /'   TSET rr  Set new thickness and camber'
     & /'   HIGH rr  Move camber and thickness highpoints'
     & /'  .CAMB     Modify camber shape directly or via loading'
     &//'   BEND     Display structural properties of buffer airfoil'
     &//'   Flap rrr Deflect trailing edge flap'
     &//'   Modi     Modify contour via cursor'
     & /'   SLOP     Toggle modified-contour slope matching flag'
     &//'   CORN     Double point with cursor (set sharp corner)'
     & /'   ADDP     Add    point with cursor or keyboard x,y'
     & /'   MOVP     Move   point with cursor or keyboard x,y'
     & /'   DELP     Delete point with cursor'
     & /'   NMOV r   Move all points in surface-normal direction'
     &//'   UNIT     Normalize buffer airfoil to unit chord'
     & /'   Dist     Determine distance between 2 cursor points'
     & /'   CLIS     List curvatures'
     & /'   CPLO     Plot curvatures'
     & /'   CANG     List panel corner angles'
     & /'   CADD ri. Add points at corners exceeding angle threshold'
     &//'   Plot     Replot buffer airfoil'
     & /'   INPL     Replot buffer airfoil without scaling (in inches)'
     & /'   Blow     Blowup plot region'
     & /'   Rese     Reset plot scale and origin'
     & /'   Wind     Plot window adjust via cursor and keys'
     &//'   TSIZ r   Change tick-mark size'
     & /'   TICK     Toggle node tick-mark plotting'
     & /'   GRID     Toggle grid plotting'
     & /'   GPAR     Toggle geometric parameter plotting'
     & /'   Over f   Overlay disk file airfoil'
     &//'   SIZE r   Change absolute plot-object size'
     & /'  .ANNO     Annotate plot'
     & /'   HARD     Hardcopy current plot'
     &//'   NAME s   Specify new airfoil name'
     & /'   NINC     Increment name version number')

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
      ELSEIF(COMAND.EQ.'GSET') THEN
       NB = N
       DO I=1, NB
         XB(I) = X(I)
         YB(I) = Y(I)
       ENDDO
       LGSAME = .TRUE.
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
       CALL PLTINI
       CALL PLOTG
       IF(LGSYM) CALL ZERCAM
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'EXEC' .OR.
     &       COMAND.EQ.'X   '      ) THEN
       CALL ABCOPY(.TRUE.)
cc       CALL NAMMOD(NAME,1,1)
cc       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SYMM') THEN
       LGSYM = .NOT.LGSYM
       IF(LGSYM) THEN
        WRITE(*,*) 'y-symmetry forcing enabled.'
        CALL ZERCAM
       ELSE
        WRITE(*,*) 'y-symmetry forcing disabled.'
       ENDIF
C
C=================================================
C---- rotate airfoil by degrees
      ELSEIF(COMAND.EQ.'ADEG' .OR.
     &       COMAND.EQ.'ARAD'     ) THEN
       IF(COMAND.EQ.'ADEG') THEN
         IF(NINPUT.GE.1) THEN
          ADEG = RINPUT(1)
         ELSE
          ADEG = 0.0
          CALL ASKR('Enter angle change (deg)^',ADEG)
         ENDIF
         ARAD = ADEG*PI/180.0
       ELSE
         IF(NINPUT.GE.1) THEN
          ARAD = RINPUT(1)
         ELSE
          ARAD = 0.0
          CALL ASKR('Enter angle change (rad)^',ARAD)
         ENDIF
       ENDIF
C
       CALL ROTATE(XB,YB,NB,ARAD)
CCC      CALL SCALC(XB,YB,SB,NB)
       CALL SEGSPL(XB,XBP,SB,NB)
       CALL SEGSPL(YB,YBP,SB,NB)
C
       APX1BA = APX1BA - ARAD
       APX2BA = APX2BA - ARAD
       APX1BT = APX1BT - ARAD
       APX2BT = APX2BT - ARAD
C
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TRAN' .OR.
     &       COMAND.EQ.'T   '      ) THEN
       IF    (NINPUT.GE.2) THEN
        DELX = RINPUT(1)
        DELY = RINPUT(2)
       ELSEIF(NINPUT.GE.1) THEN
        DELX = RINPUT(1)
        DELY = 0.0
        CALL ASKR('Enter delta(y)^',DELY)
       ELSE
        DELX = 0.0
        CALL ASKR('Enter delta(x)^',DELX)
        DELY = 0.0
        CALL ASKR('Enter delta(y)^',DELY)
       ENDIF
       DO I=1, NB
         XB(I) = XB(I) + DELX
         YB(I) = YB(I) + DELY
       ENDDO
C
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SCAL' .OR.
     &       COMAND.EQ.'S   '      ) THEN
       IF(NINPUT.GE.1) THEN
        FAC = RINPUT(1)
        XXFAC = FAC
        YYFAC = FAC
       ELSE
        FAC = 1.0
        CALL ASKR('Enter scale factor (0 for separate x,y scales)^',FAC)
        XXFAC = FAC
        YYFAC = FAC
       ENDIF
C
       IF(FAC .EQ. 0.0) THEN
        IF(NINPUT.GE.3) THEN
         XXFAC = RINPUT(2)
         YYFAC = RINPUT(3)
        ELSE
         XXFAC = 1.0
         CALL ASKR('Enter x scale factor^',XXFAC)
         YYFAC = 1.0
         CALL ASKR('Enter y scale factor^',YYFAC)
        ENDIF
       ENDIF
C
       DO I=1, NB
         XB(I) = XB(I)*XXFAC
         YB(I) = YB(I)*YYFAC
       ENDDO
C
C----- re-order if necessary to maintain counterclockwise ordering 
       IF(XXFAC*YYFAC .LT. 0.0) THEN
         DO I=1, NB/2
           XTMP = XB(I)
           YTMP = YB(I)
           XB(I) = XB(NB-I+1)
           YB(I) = YB(NB-I+1)
           XB(NB-I+1) = XTMP
           YB(NB-I+1) = YTMP
         ENDDO
       ENDIF
C
C----- re-spline new geometry
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
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'LINS') THEN
 40    CONTINUE
       IF(NINPUT.GE.4) THEN
         XOC1  = RINPUT(1)
         YFAC1 = RINPUT(2)
         XOC2  = RINPUT(3)
         YFAC2 = RINPUT(4)
       ELSE
 1001    FORMAT(/1X,A,$)
 41      WRITE(*,1001) 'Location 1...  enter  x/c, y-scale :  '
         READ(*,*,ERR=41) XOC1, YFAC1
 42      WRITE(*,1001) 'Location 2...  enter  x/c, y-scale :  '
         READ(*,*,ERR=42) XOC2, YFAC2
       ENDIF
C
       IF(ABS(XOC1-XOC2) .LT. 1.0E-5) THEN
        WRITE(*,*) 'x/c locations 1 and 2 must be different'
        NINPUT = 0
        GO TO 40
       ENDIF
C
       CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
       XLE = SEVAL(SBLE,XB,XBP,SB,NB)
       YLE = SEVAL(SBLE,YB,YBP,SB,NB)
       XTE = 0.5*(XB(1) + XB(NB))
       YTE = 0.5*(YB(1) + YB(NB))
       DO I=1, NB
         XOC = (XB(I)-XLE) / (XTE-XLE)
         FR1 = (XOC2-XOC )/(XOC2-XOC1)
         FR2 = (XOC -XOC1)/(XOC2-XOC1)
         YYFAC = FR1*YFAC1 + FR2*YFAC2
         YB(I) = YB(I)*YYFAC
       ENDDO
C
C----- re-spline new geometry
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
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'DERO') THEN
       CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
       XLE = SEVAL(SBLE,XB,XBP,SB,NB)
       YLE = SEVAL(SBLE,YB,YBP,SB,NB)
       XTE = 0.5*(XB(1) + XB(NB))
       YTE = 0.5*(YB(1) + YB(NB))
C
       ARAD = ATAN2(YTE-YLE,XTE-XLE)
       CALL ROTATE(XB,YB,NB,ARAD)
       WRITE(*,1080) ARAD / DTOR
 1080  FORMAT(/'Rotating buffer airfoil by ',F8.3,' deg.')
C
       CALL SCALC(XB,YB,SB,NB)
       CALL SEGSPL(XB,XBP,SB,NB)
       CALL SEGSPL(YB,YBP,SB,NB)
       CALL GEOPAR(XB,XBP,YB,YBP,SB,NB,W1,
     &             SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &             EI11BA,EI22BA,APX1BA,APX2BA,
     &             EI11BT,EI22BT,APX1BT,APX2BT,
     &             THICKB,CAMBRB )
C
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TGAP') THEN
       CALL TGAP(RINPUT,NINPUT)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'LERA') THEN
       CALL LERAD(RINPUT,NINPUT)
C
C--------------------------------------------------------
cc      ELSEIF(COMAND.EQ.'TC  ') THEN
cc       CALL TCBUF
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TCPL') THEN
       LPLCAM = .NOT.LPLCAM
       CALL PLTINI
       CALL GOFINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TFAC') THEN
       IF(.NOT.LPLCAM) THEN
        WRITE(*,*) 'Enabling camber,thickness plotting'
        LPLCAM = .TRUE.
        CALL PLTINI
        CALL GOFINI
        CALL PLOTG
       ENDIF
       CALL TCSCAL(RINPUT,NINPUT)
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       CALL PLTCAM('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TSET') THEN
       IF(.NOT.LPLCAM) THEN
        WRITE(*,*) 'Enabling camber,thickness plotting'
        LPLCAM = .TRUE.
        CALL PLTINI
        CALL GOFINI
        CALL PLOTG
       ENDIF
       CALL TCSET(RINPUT,NINPUT)
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       CALL PLTCAM('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'HIGH') THEN
       IF(.NOT.LPLCAM) THEN
        WRITE(*,*) 'Enabling camber,thickness plotting'
        LPLCAM = .TRUE.
        CALL PLTINI
        CALL GOFINI
        CALL PLOTG
       ENDIF
       CALL HIPNT(RINPUT,NINPUT)
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       CALL PLTCAM('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CAMB') THEN
       IF(LGSYM) THEN
        WRITE(*,*) 'Disabling symmetry enforcement.'
        LGSYM = .FALSE.
       ENDIF
       CALL CAMB
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'BEND') THEN
       CALL BENDUMP(NB,XB,YB)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CANG') THEN
       CALL CANG(XB,YB,NB,2, IMAX,AMAX)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CADD') THEN
       CALL CANG(XB,YB,NB,2, IMAX,AMAX)
       WRITE(*,*)
C
       XBMIN = XB(1)
       XBMAX = XB(1)
       DO I=1, NB
         XBMIN = MIN(XBMIN,XB(I))
         XBMAX = MAX(XBMAX,XB(I))
       ENDDO
C
C----- default inputs
       ATOL = 0.5*AMAX
       ISPL = 1
       XRF(1) = XBMIN - 0.1*(XBMAX-XBMIN)
       XRF(2) = XBMAX + 0.1*(XBMAX-XBMIN)
C
       IF    (NINPUT.LE.0) THEN
        GO TO 70
       ELSEIF(NINPUT.LE.1) THEN
        ATOL = RINPUT(1)
        GO TO 71
       ELSEIF(NINPUT.LE.2) THEN
        ATOL = RINPUT(1)
        ISPL = IINPUT(2)
        GO TO 72
       ELSEIF(NINPUT.LE.4) THEN
        ATOL = RINPUT(1)
        ISPL = IINPUT(2)
        XRF(1) = RINPUT(3)
        XRF(2) = RINPUT(4)
        GO TO 74
       ENDIF
C
 70    WRITE(*,1090) ATOL
 1090  FORMAT(1X,
     &   'Enter corner angle criterion for refinement (deg):', F8.3)
       CALL READR(1,ATOL,ERROR)
       IF(ERROR) GO TO 70
C
 71    WRITE(*,1091) ISPL
 1091  FORMAT(1X,
     &   'Enter type of spline parameter (1=uniform, 2=arclength):', I4)
       CALL READI(1,ISPL,ERROR)
       IF(ERROR) GO TO 71
       IF(ISPL.LE.0) GO TO 500
       IF(ISPL.GT.2) GO TO 71
C
 72    WRITE(*,1092) XRF(1), XRF(2)
 1092  FORMAT(1X,
     &   'Enter refinement x limits:', 2F10.5)
       CALL READR(2,XRF,ERROR)
       IF(ERROR) GO TO 72
C
 74    CONTINUE
       IF(ISPL.EQ.1) THEN
        SB(1) = 0.0
        DO I = 2, NB
          IF(XB(I).EQ.XB(I-1) .AND. YB(I).EQ.YB(I-1)) THEN
           SB(I) = SB(I-1)
          ELSE
           SB(I) = SB(I-1) + 1.0
          ENDIF
        ENDDO
        CALL SEGSPL(XB,XBP,SB,NB)
        CALL SEGSPL(YB,YBP,SB,NB)
       ENDIF
C
       CALL AREFINE(XB,YB,SB,XBP,YBP,NB, ATOL, 
     &             IBX,NNEW,W1,W2,XRF(1),XRF(2))
C
       NBADD = NNEW - NB
       WRITE(*,*) 'Number of points added: ', NBADD
C
       NB = NNEW
       DO I = 1, NB
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
     &             SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &             EI11BA,EI22BA,APX1BA,APX2BA,
     &             EI11BT,EI22BT,APX1BT,APX2BT,
     &             THICKB,CAMBRB )
C
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
C
       CALL CANG(XB,YB,NB,1, IMAX,AMAX)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CLIS') THEN
       CALL CLIS(XB,XBP,YB,YBP,SB,NB)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CPLO') THEN
       CALL PLTCRV(SBLE,XB,XBP,YB,YBP,SB,NB,W1)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'FLAP' .OR.
     &       COMAND.EQ.'F   '      ) THEN
       CALL FLAP(RINPUT,NINPUT)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'MODI' .OR.
     &       COMAND.EQ.'M   '      ) THEN
C----- plot current geometry if it's not on the screen
       IF(.NOT.LGEOPL) THEN
        CALL PLTINI
        CALL PLOTG
       ENDIF
C
       IF(LGSYM) THEN
         DO I = 1, NB
           XBT(I) = XB(I)
           YBT(I) = YB(I)
         ENDDO
       ENDIF
C
       IBFRST = 1
       IBLAST = NB
       NSIDE = 1
       XBOX(1) = XMARG
       XBOX(2) = XPAGE-XMARG
       YBOX(1) = YMARG
       YBOX(2) = YPAGE-YMARG
       LMODPL = .FALSE.
       CALL MODIXY(IBX,IBFRST,IBLAST,NSIDE,
     &             XB,YB,XBP,YBP,SB, LGSLOP,
     &             IGMOD1,IGMOD2,ISMOD,
     &             XBOX,YBOX, XBOX,YBOX,SIZE,
     &             XOFF,YOFF,XSF,YSF, LMODPL,
     &             NEWPLOTG)
C
       IF(LGSYM) THEN
         DO I = 1, NB
           XBDEL = XB(I) - XBT(I)
           YBDEL = YB(I) - YBT(I)
           XB(I) = XB(I) + XBDEL
           YB(I) = YB(I) + YBDEL
         ENDDO
         CALL ZERCAM
       ENDIF
       LGSAME = .FALSE.
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
      CALL NEWPEN(2)
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      CALL PLNEWP('magenta')
      LGEOPL = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SLOP') THEN
       LGSLOP = .NOT.LGSLOP
       IF(LGSLOP) THEN
        WRITE(*,*) 'Modified segment will be',
     &             ' made tangent at endpoints'
       ELSE
        WRITE(*,*) 'Modified segment will not be',
     &             ' made tangent at endpoints'
       ENDIF
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'CORN') THEN
       IF(NB.EQ.2*IQX) THEN
        WRITE(*,*)
     &    'Buffer airfoil arrays will overflow.  No action taken.'
         GO TO 500
       ENDIF
C
       XWS = XWIND/SIZE
       YWS = YWIND/SIZE
       CALL POINTF(XB,XBP,YB,YBP,SB,NB, XWS,YWS, XOFF,YOFF,XSF,YSF,
     &             IPNT,XC,YC)
       IF(IPNT.EQ.0) GO TO 500
       IF(IPNT.EQ.1 .OR. IPNT.EQ.NB) THEN
        WRITE(*,*) 'Cannot double trailing edge point. No action taken.'
        GO TO 500
       ENDIF
C
C----- add doubled point
       DO I=NB, IPNT, -1
         XB(I+1) = XB(I)
         YB(I+1) = YB(I)
       ENDDO
       NB = NB+1
       LGSAME = .FALSE.
C
C----- spline new geometry
       CALL SCALC(XB,YB,SB,NB)
       CALL SEGSPL(XB,XBP,SB,NB)
       CALL SEGSPL(YB,YBP,SB,NB)
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'ADDP') THEN
       CALL ADDP
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'DELP') THEN
       CALL DELP
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'NMOV') THEN
       IF(NINPUT.GE.1) THEN
        DELN = RINPUT(1)
       ELSE
        DELN = 0.0
        CALL ASKR('Enter normal movement (+ outward)^',DELN)
       ENDIF

       DO IB = 1, NB
         ENX =  YBP(IB)
         ENY = -XBP(IB)
         ENS = SQRT(ENX**2 + ENY**2)
         ENX = ENX/ENS
         ENY = ENY/ENS
         XB(IB) = XB(IB) + ENX*DELN
         YB(IB) = YB(IB) + ENY*DELN
       ENDDO

C----- re-spline new geometry
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
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
       LGSAME = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'MOVP') THEN
       CALL MOVP(NEWPLOTG)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'UNIT') THEN
       CALL NORM(XB,XBP,YB,YBP,SB,NB)
       LGSAME = .FALSE.
C
C----- re-spline new geometry
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
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'DIST' .OR.
     &       COMAND.EQ.'D   '      ) THEN
       CALL DIST
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'HARD') THEN
       IF(LPLOT) CALL PLEND
       LPLOT = .FALSE.
       CALL REPLOT(IDEVRP)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'PLOT' .OR.
     &       COMAND.EQ.'P   '      ) THEN
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'INPL') THEN
       CALL PLTINI
       XOFF0 = XOFF
       YOFF0 = YOFF
       XSF0 = XSF
       YSF0 = YSF
C
       XSF = 1.0/SIZE
       YSF = 1.0/SIZE
c       write(*,*) 'Enter Xoff, Yoff'
c       read (*,*) xoff, yoff
c       xoff = -xoff
c       yoff = -yoff
c
       CALL PLOTG
       XOFF = XOFF0
       YOFF = YOFF0
       XSF = XSF0
       YSF = YSF0
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'BLOW' .OR.
     &       COMAND.EQ.'B   '      ) THEN
       XWS = XWIND/SIZE
       YWS = YWIND/SIZE
       CALL OFFGET(XOFF,YOFF,XSF,YSF,XWS,YWS, .TRUE. , .TRUE. )
       CALL GOFSET
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'RESE' .OR.
     &       COMAND.EQ.'R   '      ) THEN
       CALL PLTINI
       CALL GOFINI
       CALL PLOTG
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
 80    CALL PLTINI
       CALL PLOTG
C
       CALL GETCURSORXY(XCRS,YCRS,CHKEY)
C
C----- do possible pan,zoom operations based on CHKEY
       CALL KEYOFF(XCRS,YCRS,CHKEY, XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
       IF(LPLNEW) THEN
        CALL GOFSET
        GO TO 80
       ENDIF
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TSIZ') THEN
       IF(NINPUT.GE.1) THEN
        GTICK = RINPUT(1)
       ELSE
        WRITE(*,*)
     &    'Current tick-mark size (as fraction of perimeter) =', GTICK
        CALL ASKR('Enter new tick-mark size^',GTICK)
       ENDIF
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'TICK') THEN
       LGTICK = .NOT.LGTICK
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'GRID') THEN
       LGGRID = .NOT.LGGRID
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'GPAR') THEN
       LGPARM = .NOT.LGPARM
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SIZE') THEN
       IF(NINPUT.GE.1) THEN
        SIZE = RINPUT(1)
       ELSE
        WRITE(*,*) 'Current plot-object size =', SIZE
        CALL ASKR('Enter new plot-object size^',SIZE)
       ENDIF
       CALL PLTINI
       CALL PLOTG
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'OVER' .OR.
     &       COMAND.EQ.'O   '      ) THEN
       CALL OVER(COMARG)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'ANNO') THEN
       IF(LPLOT) THEN
        CALL ANNOT(CH)
       ELSE
        WRITE(*,*) 'No active plot to annotate'
       ENDIF
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'NAME') THEN
       IF(COMARG.EQ.' ') THEN
        CALL NAMMOD(NAME,0,-1)
       ELSE
        NAME = COMARG
       ENDIF
       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'NINC') THEN
       CALL NAMMOD(NAME,1,1)
       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'NDEC') THEN
       CALL NAMMOD(NAME,-1,1)
       CALL STRIP(NAME,NNAME)
C
C--------------------------------------------------------
      ELSEIF(COMAND.EQ.'SINT') THEN
       CALL SPLNXY(XB,XBP,YB,YBP,SB,NB)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'cyan')
       CALL NEWPEN(2)
       CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
       CALL PLNEWP('magenta')
       LGEOPL = .FALSE.
C
C--------------------------------------------------------
      ELSE
       WRITE(*,1100) COMAND
 1100  FORMAT(' Command ',A4,' not recognized.  Type a " ? " for list.')
       COMAND = '****'
      ENDIF
C
      GO TO 500
      END ! GDES


      SUBROUTINE NEWPLOTG
      CALL GOFSET
      CALL PLTINI
      CALL PLOTG
      RETURN
      END


      SUBROUTINE ABCOPY(LCONF)
      INCLUDE 'XFOIL.INC'
      LOGICAL LCONF
C
      IF(NB.LE.1) THEN
       WRITE(*,*) 'ABCOPY: Buffer airfoil not available.'
       RETURN
      ELSEIF(NB.GT.IQX-5) THEN
       WRITE(*,*) 'Maximum number of panel nodes  : ',IQX-5
       WRITE(*,*) 'Number of buffer airfoil points: ',NB
       WRITE(*,*) 'Current airfoil cannot be set.'
       WRITE(*,*) 'Try executing PANE at Top Level instead.'
       RETURN
      ENDIF
      IF(N.NE.NB) LBLINI = .FALSE.
C
      N = NB
      DO 101 I=1, N
        X(I) = XB(I)
        Y(I) = YB(I)
  101 CONTINUE
      LGSAME = .TRUE.
C
      IF(LBFLAP) THEN
       XOF = XBF
       YOF = YBF
       LFLAP = .TRUE.
      ENDIF
C
C---- strip out doubled points
      I = 1
 102  CONTINUE
      I = I+1
      IF(X(I-1).EQ.X(I) .AND. Y(I-1).EQ.Y(I)) THEN
        DO 104 J=I, N-1
          X(J) = X(J+1)
          Y(J) = Y(J+1)
 104    CONTINUE
        N = N-1
      ENDIF
      IF(I.LT.N) GO TO 102
C
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)

      CALL NCALC(X,Y,S,N,NX,NY)

      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD  = SQRT( (XTE-XLE)**2 + (YTE-YLE)**2 )

      CALL TECALC
      CALL APCALC
C
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
C
      IF(LCONF) WRITE(*,1200) N
 1200 FORMAT(/' Current airfoil nodes set from buffer airfoil nodes (',
     &        I4,' )')
C
      RETURN
      END ! ABCOPY


      SUBROUTINE GOFINI
C----------------------------------------------------------
C     Sets initial airfoil scaling and offset parameters   
C----------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- get airfoil bounding box
      XBMIN = XB(1)
      YBMIN = YB(1)
      XBMAX = XB(1)
      YBMAX = YB(1)
      DO I=1, NB
        XBMIN = MIN(XBMIN,XB(I))
        YBMIN = MIN(YBMIN,YB(I))
        XBMAX = MAX(XBMAX,XB(I))
        YBMAX = MAX(YBMAX,YB(I))
      ENDDO
C
C---- set camber and thickness distributions
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &             XB,XBP,YB,YBP,SB,NB )
C      
C---- get camber,thickness y bounds
      CMMIN = 0.
      CMMAX = 0.
      DO I=1, NCM
        CMMIN = MIN(CMMIN,YCM(I))
        CMMAX = MAX(CMMAX,YCM(I))
      ENDDO
      TKMIN = 0.
      TKMAX = 0.
      DO I=1, NTK
        TKMIN = MIN(TKMIN,YTK(I))
        TKMAX = MAX(TKMAX,YTK(I))
      ENDDO
C
      XRANGE = XBMAX - XBMIN
      YRANGE = YBMAX - YBMIN
C
C---- set x,y scaling factors needed for O(1) size plot with "nice" limits
      CALL SCALIT(1,0.95*XRANGE,0.0,XSF)
      CALL SCALIT(1,0.95*YRANGE,0.0,YSF)
C  
C---- grid increment as a fraction of a nice upper bound on delta x
cc      DXYG = 0.1 / XSF
      DXYG = 0.1 / MIN(XSF,YSF)
C  
C---- set "nice" grid limits as integer multiples of DXYG
c      XGMAX = DXYG*(INT(XBMAX/DXYG+1000.05) - 999)
c      XGMIN = DXYG*(INT(XBMIN/DXYG-1000.05) + 999)
c      YGMAX = DXYG*(INT(YBMAX/DXYG+1000.25) - 999)
c      YGMIN = DXYG*(INT(YBMIN/DXYG-1000.25) + 999)
C  
C---- set "nice" grid limits as integer multiples of DXYG
      XGMAX = DXYG*(INT(XBMAX/DXYG+1001.01) - 1000)
      XGMIN = DXYG*(INT(XBMIN/DXYG-1001.01) + 1000)
      YGMAX = DXYG*(INT(YBMAX/DXYG+1001.01) - 1000)
      YGMIN = DXYG*(INT(YBMIN/DXYG-1001.01) + 1000)
C
C---- set bounding box for thickness/camber plot
      DXYC = DXYG
      XCMIN = XGMIN
      XCMAX = XGMAX
      YCMIN = MIN(CMMIN,-TKMAX)
      YCMAX = MAX(CMMAX, TKMAX)
      YCMAX = DXYC*(INT(YCMAX/DXYC+1000.25) - 999)
      YCMIN = DXYC*(INT(YCMIN/DXYC-1000.25) + 999)
      YCMAX = MAX(YCMAX,YCMIN+DXYC)
C
C---- set minimum scaling factor to fit airfoil or grid
      IF(LGGRID) THEN
        XRANGE = XGMAX - XGMIN
        YRANGE = YGMAX - YGMIN
      ELSE
        XRANGE = XBMAX - XBMIN
        YRANGE = YBMAX - YBMIN
      ENDIF
C
C---- include y range from thickness/camber plot if present
      IF(LPLCAM) THEN
        YRANGE = YRANGE + (YCMAX - YCMIN)
      ENDIF
C
      RANGE = MAX(XRANGE,YRANGE)
C
      SF = MIN( 1.0/XRANGE , PLOTAR/YRANGE )
      XSF = SF
      YSF = SF
      CHG = 0.75*CH * RANGE*SF
C--- HHY 4/24/01 keep the character size from getting too low

      CHG = MAX(CHG,0.0075)
C
      IF(LGGRID) THEN
C------ set offsets to position grid, with space for numerical axis annotations
        XOFF = XGMIN - 0.05*RANGE - 3.0*CHG/SF
        YOFF = YGMIN - 0.05*RANGE - 2.0*CHG/SF
      ELSE
C------ set offsets to position airfoil
        XOFF = XBMIN - 0.05*RANGE
        YOFF = YBMIN - 0.05*RANGE
      ENDIF
C
C---- set plot limits for DCp plot (y-axis limit defaults set in INIT)
      XPMIN = XGMIN
      XPMAX = XGMAX
ccc      DXYP = DXYG
      CALL AXISADJ(YPMIN,YPMAX,PSPAN,DXYP,NTICS)
C
C---- set Yoffset for camber plot in scale factor YSF for geom plots
      DYOFFC = - YGMAX + YCMIN - 2.2*CHG/YSF
C
C---- set the Cp scale factor for DCp plots
      PAR = (YPAGE-2.0*YMARG)/(XPAGE-2.0*XMARG)
      DPRANGE = YPMAX-YPMIN
      DYPLT = MAX(0.1,PAR-PLOTAR)
      YSFP = 0.8*DYPLT/DPRANGE
      YSFP = YSFP/YSF
C
C---- set shifts to YOFF for DCp plots in scale factor YSF for geom plots
      DYOFFP = -YCMAX+DYOFFC + YPMIN*YSFP - 2.2*CHG/YSF
C
      RETURN
      END ! GOFINI  



      SUBROUTINE GOFSET
C----------------------------------------------------------
C     Sets grid-overlay parameters
C----------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
C---- airfoil extent
      XBMIN = XB(1)
      YBMIN = YB(1)
      XBMAX = XB(1)
      YBMAX = YB(1)
      DO I=1, NB
        XBMIN = MIN(XBMIN,XB(I))
        YBMIN = MIN(YBMIN,YB(I))
        XBMAX = MAX(XBMAX,XB(I))
        YBMAX = MAX(YBMAX,YB(I))
      ENDDO
C
      RANGE = MAX( (XWIND/SIZE)/XSF , (YWIND/SIZE)/YSF )
C
C---- set bounding-box corner locations in user coordinates
      XG1 = XOFF + 0.1*RANGE + 4.0*CHG/XSF
      YG1 = YOFF + 0.1*RANGE + 2.0*CHG/YSF
      XG2 = XOFF - 0.1*RANGE + (XWIND/SIZE)/XSF
      YG2 = YOFF - 0.1*RANGE + (YWIND/SIZE)/YSF
C
C---- crunch down onto airfoil limits
      XG1 = MAX(XG1,XBMIN)
      XG2 = MIN(XG2,XBMAX)
      YG1 = MAX(YG1,YBMIN)
      YG2 = MIN(YG2,YBMAX)
C
C---- set x,y scaling factors needed for O(1) size plot with "nice" limits
      CALL SCALIT(1,0.95*(XG2-XG1),0.0,GXSF)
      CALL SCALIT(1,0.95*(YG2-YG1),0.0,GYSF)
C  
      GSF = GXSF
ccc   GSF = MIN(GXSF,GYSF)
C
C---- grid increment as a fraction of a nice upper bound on delta x
      DXYG = 0.1 / GSF
C  
C---- set "nice" grid limits as integer multiples of DXYG
      XGMAX = DXYG*(INT(XG2/DXYG+1001.01) - 1000)
      XGMIN = DXYG*(INT(XG1/DXYG-1001.01) + 1000)
      YGMAX = DXYG*(INT(YG2/DXYG+1001.01) - 1000)
      YGMIN = DXYG*(INT(YG1/DXYG-1001.01) + 1000)
C
      RETURN
      END ! GOFSET



      SUBROUTINE TGAP(RINPUT,NINPUT)
C----------------------------------
C     Used to set buffer airfoil 
C     trailing edge gap
C----------------------------------
      INCLUDE 'XFOIL.INC'
      DIMENSION RINPUT(*)
C
      CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
      XBLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YBLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XBTE = 0.5*(XB(1)+XB(NB))
      YBTE = 0.5*(YB(1)+YB(NB))
      CHBSQ = (XBTE-XBLE)**2 + (YBTE-YBLE)**2
C
      DXN = XB(1) - XB(NB)
      DYN = YB(1) - YB(NB)
      GAP = SQRT(DXN**2 + DYN**2)
C
C---- components of unit vector parallel to TE gap
      IF(GAP.GT.0.0) THEN
       DXU = DXN / GAP
       DYU = DYN / GAP
      ELSE
       DXU = -.5*(YBP(NB) - YBP(1))
       DYU = 0.5*(XBP(NB) - XBP(1))
      ENDIF
C
      IF    (NINPUT .GE. 2) THEN
       GAPNEW = RINPUT(1)
       DOC    = RINPUT(2)
      ELSEIF(NINPUT .GE. 1) THEN
       GAPNEW = RINPUT(1)
       DOC = 1.0
       CALL ASKR('Enter blending distance/c (0..1)^',DOC)
      ELSE
       WRITE(*,1000) GAP
 1000  FORMAT(/' Current gap =',F9.5)
       GAPNEW = 0.0
       CALL ASKR('Enter new gap^',GAPNEW)
       DOC = 1.0
       CALL ASKR('Enter blending distance/c (0..1)^',DOC)
      ENDIF
C
      DOC = MIN( MAX( DOC , 0.0 ) , 1.0 )
C
      DGAP = GAPNEW - GAP
C
C---- go over each point, changing the y-thickness appropriately
      DO 30 I=1, NB
C
C------ chord-based x/c
        XOC = (  (XB(I)-XBLE)*(XBTE-XBLE)
     &         + (YB(I)-YBLE)*(YBTE-YBLE) ) / CHBSQ
C
C------ thickness factor tails off exponentially away from trailing edge
        IF(DOC .EQ. 0.0) THEN
          TFAC = 0.0
          IF(I.EQ.1 .OR. I.EQ.NB) TFAC = 1.0
        ELSE
          ARG = MIN( (1.0-XOC)*(1.0/DOC-1.0) , 15.0 )
          TFAC = EXP(-ARG)
        ENDIF
C
        IF(SB(I).LE.SBLE) THEN
         XB(I) = XB(I) + 0.5*DGAP*XOC*TFAC*DXU
         YB(I) = YB(I) + 0.5*DGAP*XOC*TFAC*DYU
        ELSE
         XB(I) = XB(I) - 0.5*DGAP*XOC*TFAC*DXU
         YB(I) = YB(I) - 0.5*DGAP*XOC*TFAC*DYU
        ENDIF
   30 CONTINUE
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
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      CALL PLNEWP('magenta')
C
      LGEOPL = .FALSE.
C
      RETURN
      END ! TGAP



      SUBROUTINE LERAD(RINPUT,NINPUT)
C----------------------------
C     Changes buffer airfoil 
C     leading edge radius.
C----------------------------
      INCLUDE 'XFOIL.INC'
      DIMENSION RINPUT(*)
C
      IF    (NINPUT .GE. 2) THEN
       RFAC = RINPUT(1)
       DOC  = RINPUT(2)
      ELSEIF(NINPUT .GE. 1) THEN
       RFAC = RINPUT(1)
       DOC = 1.0
       CALL ASKR('Enter blending distance/c from LE^',DOC)
      ELSE
       RFAC = 1.0
       CALL ASKR('Enter approx. new/old LE radius scaling ratio^',RFAC)
       DOC = 1.0
       CALL ASKR('Enter blending distance/c from LE^',DOC)
      ENDIF
C
      DOC = MAX( DOC , 0.001 )
C
      CALL LERSCL(XB,XBP,YB,YBP,SB,NB, DOC,RFAC, W1,W2)
C
      DO 40 I=1, NB
        XB(I) = W1(I)
        YB(I) = W2(I)
   40 CONTINUE
      LGSAME = .FALSE.
C
C---- spline new coordinates
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
C---- find max curvature
      CVMAX = 0.
      DO 6 I=NB/4, (3*NB)/4
        CV = CURV(SB(I),XB,XBP,YB,YBP,SB,NB)
        CVMAX = MAX( ABS(CV) , CVMAX )
    6 CONTINUE
C
      RADIUS = 1.0/CVMAX
C
      WRITE(*,1000) RADIUS
 1000 FORMAT(/' New LE radius = ',F7.5)
C
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      CALL PLNEWP('magenta')
C
      LGEOPL = .FALSE.
C
      RETURN
      END ! LERAD



      SUBROUTINE SCLXY
C---------------------------------------------------
C     Scale airfoil about LE, TE, or selected point 
C---------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*1 VAR
C
      CALL LEFIND(SBLE,XB,XBP,YB,YBP,SB,NB)
      XLE = SEVAL(SBLE,XB,XBP,SB,NB)
      YLE = SEVAL(SBLE,YB,YBP,SB,NB)
      XTE = 0.5*(XB(1) + XB(NB))
      YTE = 0.5*(YB(1) + YB(NB))
C
      WRITE(*,*) 'Enter origin for airfoil scaling:'
      WRITE(*,*) '  L  scales about LE'
      WRITE(*,*) '  T  scales about TE'
      WRITE(*,*) '  P  scales about input point'
C      
      CALL ASKS('Select origin for scaling^',VAR)
      IF (VAR.EQ.'L') THEN
        XORG = XLE
        YORG = YLE
       ELSE IF (VAR.EQ.'T') THEN
        XORG = XTE
        YORG = YTE
       ELSE 
        XORG = 0.25
        YORG = 0.0
        CALL ASKR('Enter X origin for scaling^',XORG)
        CALL ASKR('Enter Y origin for scaling^',YORG)
      ENDIF       
C      
      SCL = 1.0
      CALL ASKR('Enter scaling factor about selected point^',SCL)
C
      DO 10 I=1, NB
        XB(I) = SCL*(XB(I) - XORG) + XORG 
        YB(I) = SCL*(YB(I) - YORG) + YORG 
   10 CONTINUE
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
      END ! SCLXY



      SUBROUTINE FLAP(RINPUT,NINPUT)
C----------------------------------------------------
C     Modifies buffer airfoil for a deflected flap.
C     Points may be added/subtracted in the flap
C     break vicinity to clean things up.
C----------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LCHANGE
      DIMENSION RINPUT(*)
C
      LOGICAL INSID
      LOGICAL INSIDE
      LOGICAL LT1NEW,LT2NEW,LB1NEW,LB2NEW
C
      SHT = CH * MAX(XSF,YSF)
C
      IF(NINPUT.GE.2) THEN
       XBF = RINPUT(1)
       YBF = RINPUT(2)
      ELSE
       XBF = -999.0
       YBF = -999.0
      ENDIF
C
      CALL GETXYF(XB,XBP,YB,YBP,SB,NB, TOPS,BOTS,XBF,YBF)
      INSID = INSIDE(XB,YB,NB,XBF,YBF)
C
      WRITE(*,1050) XBF, YBF
 1050 FORMAT(/' Flap hinge: x,y =', 2F9.5 )
C
      IF(NINPUT.GE.3) THEN
       DDEF = RINPUT(3)
      ELSE
       DDEF = 0.0
       CALL ASKR('Enter flap deflection in degrees (+ down)^',DDEF)
      ENDIF
      RDEF = DDEF*PI/180.0
      IF(RDEF .EQ. 0.0) RETURN
C
C
      IF(INSID) THEN
        ATOP = MAX( 0.0 , -RDEF )
        ABOT = MAX( 0.0 ,  RDEF )
      ELSE
        CHX = DEVAL(BOTS,XB,XBP,SB,NB) - DEVAL(TOPS,XB,XBP,SB,NB)
        CHY = DEVAL(BOTS,YB,YBP,SB,NB) - DEVAL(TOPS,YB,YBP,SB,NB)
        FVX = SEVAL(BOTS,XB,XBP,SB,NB) + SEVAL(TOPS,XB,XBP,SB,NB)
        FVY = SEVAL(BOTS,YB,YBP,SB,NB) + SEVAL(TOPS,YB,YBP,SB,NB)
        CRSP = CHX*(YBF-0.5*FVY) - CHY*(XBF-0.5*FVX)
        IF(CRSP .GT. 0.0) THEN
C-------- flap hinge is above airfoil
          ATOP = MAX( 0.0 ,  RDEF )
          ABOT = MAX( 0.0 ,  RDEF )
        ELSE
C-------- flap hinge is below airfoil
          ATOP = MAX( 0.0 , -RDEF )
          ABOT = MAX( 0.0 , -RDEF )
        ENDIF
      ENDIF
C
C---- find upper and lower surface break arc length values...
      CALL SSS(TOPS,ST1,ST2,ATOP,XBF,YBF,XB,XBP,YB,YBP,SB,NB,1)
      CALL SSS(BOTS,SB1,SB2,ABOT,XBF,YBF,XB,XBP,YB,YBP,SB,NB,2)
C
C---- ... and x,y coordinates
      XT1 = SEVAL(ST1,XB,XBP,SB,NB)
      YT1 = SEVAL(ST1,YB,YBP,SB,NB)
      XT2 = SEVAL(ST2,XB,XBP,SB,NB)
      YT2 = SEVAL(ST2,YB,YBP,SB,NB)
      XB1 = SEVAL(SB1,XB,XBP,SB,NB)
      YB1 = SEVAL(SB1,YB,YBP,SB,NB)
      XB2 = SEVAL(SB2,XB,XBP,SB,NB)
      YB2 = SEVAL(SB2,YB,YBP,SB,NB)
C
C
      WRITE(*,1100) XT1, YT1, XT2, YT2,
     &              XB1, YB1, XB2, YB2
 1100 FORMAT(/' Top breaks: x,y =  ', 2F9.5, 4X, 2F9.5
     &       /' Bot breaks: x,y =  ', 2F9.5, 4X, 2F9.5)
C
C---- find points adjacent to breaks
      DO 5 I=1, NB-1
        IF(SB(I).LE.ST1 .AND. SB(I+1).GT.ST1) IT1 = I+1
        IF(SB(I).LT.ST2 .AND. SB(I+1).GE.ST2) IT2 = I
        IF(SB(I).LE.SB1 .AND. SB(I+1).GT.SB1) IB1 = I
        IF(SB(I).LT.SB2 .AND. SB(I+1).GE.SB2) IB2 = I+1
    5 CONTINUE
C
      DSAVG = (SB(NB)-SB(1))/FLOAT(NB-1)
C
C---- smallest fraction of s increments i+1 and i+2 away from break point
      SFRAC = 0.33333
C
      IF(ATOP .NE. 0.0) THEN
        ST1P = ST1 + SFRAC*(SB(IT1  )-ST1)
        ST1Q = ST1 + SFRAC*(SB(IT1+1)-ST1)
        IF(SB(IT1) .LT. ST1Q) THEN
C-------- simply move adjacent point to ideal SFRAC location
          XT1NEW = SEVAL(ST1Q,XB,XBP,SB,NB)
          YT1NEW = SEVAL(ST1Q,YB,YBP,SB,NB)
          LT1NEW = .FALSE.
        ELSE
C-------- make new point at SFRAC location
          XT1NEW = SEVAL(ST1P,XB,XBP,SB,NB)
          YT1NEW = SEVAL(ST1P,YB,YBP,SB,NB)
          LT1NEW = .TRUE.
        ENDIF
C
        ST2P = ST2 + SFRAC*(SB(IT2 )-ST2)
        IT2Q = MAX(IT2-1,1)
        ST2Q = ST2 + SFRAC*(SB(IT2Q)-ST2)
        IF(SB(IT2) .GT. ST2Q) THEN
C-------- simply move adjacent point
          XT2NEW = SEVAL(ST2Q,XB,XBP,SB,NB)
          YT2NEW = SEVAL(ST2Q,YB,YBP,SB,NB)
          LT2NEW = .FALSE.
        ELSE
C-------- make new point
          XT2NEW = SEVAL(ST2P,XB,XBP,SB,NB)
          YT2NEW = SEVAL(ST2P,YB,YBP,SB,NB)
          LT2NEW = .TRUE.
        ENDIF
      ENDIF
C
      IF(ABOT .NE. 0.0) THEN
        SB1P = SB1 + SFRAC*(SB(IB1  )-SB1)
        SB1Q = SB1 + SFRAC*(SB(IB1-1)-SB1)
        IF(SB(IB1) .GT. SB1Q) THEN
C-------- simply move adjacent point
          XB1NEW = SEVAL(SB1Q,XB,XBP,SB,NB)
          YB1NEW = SEVAL(SB1Q,YB,YBP,SB,NB)
          LB1NEW = .FALSE.
        ELSE
C-------- make new point
          XB1NEW = SEVAL(SB1P,XB,XBP,SB,NB)
          YB1NEW = SEVAL(SB1P,YB,YBP,SB,NB)
          LB1NEW = .TRUE.
        ENDIF
C
        SB2P = SB2 + SFRAC*(SB(IB2 )-SB2)
        IB2Q = MIN(IB2+1,NB)
        SB2Q = SB2 + SFRAC*(SB(IB2Q)-SB2)
        IF(SB(IB2) .LT. SB2Q) THEN
C-------- simply move adjacent point
          XB2NEW = SEVAL(SB2Q,XB,XBP,SB,NB)
          YB2NEW = SEVAL(SB2Q,YB,YBP,SB,NB)
          LB2NEW = .FALSE.
        ELSE
C-------- make new point
          XB2NEW = SEVAL(SB2P,XB,XBP,SB,NB)
          YB2NEW = SEVAL(SB2P,YB,YBP,SB,NB)
          LB2NEW = .TRUE.
        ENDIF
      ENDIF
C
cc      DSTOP = ABS(SB(IT2)-SB(IT1))
cc      DSBOT = ABS(SB(IB2)-SB(IB1))
C
      SIND = SIN(RDEF)
      COSD = COS(RDEF)
C
C---- rotate flap points about the hinge point (XBF,YBF)
      DO 10 I=1, NB
        IF(I.GE.IT1 .AND. I.LE.IB1) GO TO 10
C
        XBAR = XB(I) - XBF
        YBAR = YB(I) - YBF
C
        XB(I) = XBF  +  XBAR*COSD  +  YBAR*SIND
        YB(I) = YBF  -  XBAR*SIND  +  YBAR*COSD
   10 CONTINUE
C
      IDIF = IT1-IT2-1
      IF(IDIF.GT.0) THEN
C----- delete points on upper airfoil surface which "disappeared".
       NB  = NB -IDIF
       IT1 = IT1-IDIF
       IB1 = IB1-IDIF
       IB2 = IB2-IDIF
       DO 21 I=IT2+1, NB
         SB(I) = SB(I+IDIF)
         XB(I) = XB(I+IDIF)
         YB(I) = YB(I+IDIF)
   21  CONTINUE
      ENDIF
C
      IDIF = IB2-IB1-1
      IF(IDIF.GT.0) THEN
C----- delete points on lower airfoil surface which "disappeared".
       NB  = NB -IDIF
       IB2 = IB2-IDIF
       DO 22 I=IB1+1, NB
         SB(I) = SB(I+IDIF)
         XB(I) = XB(I+IDIF)
         YB(I) = YB(I+IDIF)
   22  CONTINUE
      ENDIF
C
C
      IF(ATOP .EQ. 0.0) THEN
C
C------ arc length of newly created surface on top of airfoil
        DSNEW = ABS(RDEF)*SQRT((XT1-XBF)**2 + (YT1-YBF)**2)
C
C------ number of points to be added to define newly created surface
        NPADD = INT(1.5*DSNEW/DSAVG + 1.0)
ccc     NPADD = INT(1.5*DSNEW/DSTOP + 1.0)
C
C------ skip everything if no points are to be added
        IF(NPADD.EQ.0) GO TO 35
C
C------ increase coordinate array length to make room for the new point(s)
        NB  = NB +NPADD
        IT1 = IT1+NPADD
        IB1 = IB1+NPADD
        IB2 = IB2+NPADD
        DO 30 I=NB, IT1, -1
          XB(I) = XB(I-NPADD)
          YB(I) = YB(I-NPADD)
   30   CONTINUE
C
C------ add new points along the new surface circular arc segment
        DANG = RDEF / FLOAT(NPADD)
        XBAR = XT1 - XBF
        YBAR = YT1 - YBF
        DO 31 IP=1, NPADD
          ANG = DANG*(FLOAT(IP) - 0.5)
          CA = COS(ANG)
          SA = SIN(ANG)
C
          XB(IT1-IP) = XBF  +  XBAR*CA + YBAR*SA
          YB(IT1-IP) = YBF  -  XBAR*SA + YBAR*CA
   31   CONTINUE
C
      ELSE
C
C------ set point in the corner and possibly two adjacent points
        NPADD = 1
        IF(LT2NEW) NPADD = NPADD+1
        IF(LT1NEW) NPADD = NPADD+1
C
        NB  = NB +NPADD
        IT1 = IT1+NPADD
        IB1 = IB1+NPADD
        IB2 = IB2+NPADD
        DO 33 I=NB, IT1, -1
          XB(I) = XB(I-NPADD)
          YB(I) = YB(I-NPADD)
   33   CONTINUE
C
        IF(LT1NEW) THEN
         XB(IT1-1) = XT1NEW
         YB(IT1-1) = YT1NEW
         XB(IT1-2) = XT1
         YB(IT1-2) = YT1
        ELSE
         XB(IT1  ) = XT1NEW
         YB(IT1  ) = YT1NEW
         XB(IT1-1) = XT1
         YB(IT1-1) = YT1
        ENDIF
C
        XBAR = XT2NEW - XBF
        YBAR = YT2NEW - YBF
        IF(LT2NEW) THEN
          XB(IT2+1) = XBF  +  XBAR*COSD + YBAR*SIND
          YB(IT2+1) = YBF  -  XBAR*SIND + YBAR*COSD
        ELSE
          XB(IT2  ) = XBF  +  XBAR*COSD + YBAR*SIND
          YB(IT2  ) = YBF  -  XBAR*SIND + YBAR*COSD
        ENDIF
C
      ENDIF
   35 CONTINUE
C
C
      IF(ABOT .EQ. 0.0) THEN
C
C------ arc length of newly created surface on top of airfoil
        DSNEW = ABS(RDEF)*SQRT((XB1-XBF)**2 + (YB1-YBF)**2)
C
C------ number of points to be added to define newly created surface
        NPADD = INT(1.5*DSNEW/DSAVG + 1.0)
ccc     NPADD = INT(1.5*DSNEW/DSBOT + 1.0)
C
C------ skip everything if no points are to be added
        IF(NPADD.EQ.0) GO TO 45
C
C------ increase coordinate array length to make room for the new point(s)
        NB  = NB +NPADD
        IB2 = IB2+NPADD
        DO 40 I=NB, IB2, -1
          XB(I) = XB(I-NPADD)
          YB(I) = YB(I-NPADD)
   40   CONTINUE
C
C------ add new points along the new surface circular arc segment
        DANG = RDEF / FLOAT(NPADD)
        XBAR = XB1 - XBF
        YBAR = YB1 - YBF
        DO 41 IP=1, NPADD
          ANG = DANG*(FLOAT(IP) - 0.5)
          CA = COS(ANG)
          SA = SIN(ANG)
C
          XB(IB1+IP) = XBF  +  XBAR*CA + YBAR*SA
          YB(IB1+IP) = YBF  -  XBAR*SA + YBAR*CA
   41   CONTINUE
C
      ELSE

C------ set point in the corner and possibly two adjacent points
        NPADD = 1
        IF(LB2NEW) NPADD = NPADD+1
        IF(LB1NEW) NPADD = NPADD+1
C
        NB  = NB +NPADD
        IB2 = IB2+NPADD
        DO 43 I=NB, IB2, -1
          XB(I) = XB(I-NPADD)
          YB(I) = YB(I-NPADD)
   43   CONTINUE
C
        IF(LB1NEW) THEN
         XB(IB1+1) = XB1NEW
         YB(IB1+1) = YB1NEW
         XB(IB1+2) = XB1
         YB(IB1+2) = YB1
        ELSE
         XB(IB1  ) = XB1NEW
         YB(IB1  ) = YB1NEW
         XB(IB1+1) = XB1
         YB(IB1+1) = YB1
        ENDIF
C
        XBAR = XB2NEW - XBF
        YBAR = YB2NEW - YBF
        IF(LB2NEW) THEN
          XB(IB2-1) = XBF  +  XBAR*COSD + YBAR*SIND
          YB(IB2-1) = YBF  -  XBAR*SIND + YBAR*COSD
        ELSE
          XB(IB2  ) = XBF  +  XBAR*COSD + YBAR*SIND
          YB(IB2  ) = YBF  -  XBAR*SIND + YBAR*COSD
        ENDIF
C
      ENDIF
   45 CONTINUE
C
      LGSAME = .FALSE.
C
C
C---- check new geometry for splinter segments 
      STOL = 0.2
      CALL SCHECK(XB,YB,NB, STOL, LCHANGE)
C
C---- spline new geometry
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
      LBFLAP = .TRUE.
C
      IF(LGSYM) THEN
       WRITE(*,*)
       WRITE(*,*) 'Disabling symmetry enforcement'
       LGSYM = .FALSE.
      ENDIF
C
C
      IF(.NOT.LPLOT) THEN
       CALL PLTINI
      ENDIF
C
C---- save current color and set new color
      CALL GETCOLOR(ICOL0)
C
      CALL NEWCOLORNAME('green')
      CALL PLOT((XBF-XOFF)*XSF,(YBF-YOFF)*YSF,3)
      CALL PLOT((XT1-XOFF)*XSF,(YT1-YOFF)*YSF,2)
      CALL PLOT((XBF-XOFF)*XSF,(YBF-YOFF)*YSF,3)
      CALL PLOT((XB1-XOFF)*XSF,(YB1-YOFF)*YSF,2)
C
      IF(ATOP .EQ. 0.0) THEN
        XBAR = XT1 - XBF
        YBAR = YT1 - YBF
        XT1C = XBF  +  XBAR*COSD + YBAR*SIND
        YT1C = YBF  -  XBAR*SIND + YBAR*COSD
        CALL PLOT((XBF -XOFF)*XSF,(YBF -YOFF)*YSF,3)
        CALL PLOT((XT1C-XOFF)*XSF,(YT1C-YOFF)*YSF,2)
      ENDIF
C
      IF(ABOT .EQ. 0.0) THEN
        XBAR = XB1 - XBF
        YBAR = YB1 - YBF
        XB1C = XBF  +  XBAR*COSD + YBAR*SIND
        YB1C = YBF  -  XBAR*SIND + YBAR*COSD
        CALL PLOT((XBF -XOFF)*XSF,(YBF -YOFF)*YSF,3)
        CALL PLOT((XB1C-XOFF)*XSF,(YB1C-YOFF)*YSF,2)
      ENDIF
C
      CALL NEWCOLORNAME('red')
      CALL PLSYMB((XBF-XOFF)*XSF,(YBF-YOFF)*YSF,0.5*SHT,1,0.0,0)
C
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      CALL PLNEWP('magenta')
C
      LGEOPL = .FALSE.
C
      CALL NEWCOLOR(ICOL0)
      RETURN
      END ! FLAP


      LOGICAL FUNCTION INSIDE(X,Y,N, XF,YF)
      DIMENSION X(N),Y(N)
C-------------------------------------
C     Returns .TRUE. if point XF,YF 
C     is inside contour X(i),Y(i).
C-------------------------------------
C
C---- integrate subtended angle around airfoil perimeter
      ANGLE = 0.0
      DO 10 I=1, N
        IP = I+1
        IF(I.EQ.N) IP = 1
        XB1 = X(I)  - XF
        YB1 = Y(I)  - YF
        XB2 = X(IP) - XF
        YB2 = Y(IP) - YF
        ANGLE = ANGLE + (XB1*YB2 - YB1*XB2)
     &                   / SQRT((XB1**2 + YB1**2)*(XB2**2 + YB2**2))
 10   CONTINUE
C
C---- angle = 0 if XF,YF is outside, angle = +/- 2 pi  if XF,YF is inside
      INSIDE = ABS(ANGLE) .GT. 1.0
C
      RETURN
      END ! INSIDE



      SUBROUTINE GETXYF(X,XP,Y,YP,S,N, TOPS,BOTS,XF,YF)
      DIMENSION X(N),XP(N),Y(N),YP(N),S(N)
C
      IF(XF .EQ. -999.0)
     &  CALL ASKR('Enter flap hinge x location^',XF)
C
C---- find top and bottom y at hinge x location
      TOPS = S(1) + (X(1) - XF)
      BOTS = S(N) - (X(N) - XF)
      CALL SINVRT(TOPS,XF,X,XP,S,N)      
      CALL SINVRT(BOTS,XF,X,XP,S,N)      
      TOPY = SEVAL(TOPS,Y,YP,S,N)
      BOTY = SEVAL(BOTS,Y,YP,S,N)
C
      WRITE(*,1000) TOPY, BOTY
 1000 FORMAT(/'  Top    surface:  y =', F8.4,'     y/t = 1.0'
     &       /'  Bottom surface:  y =', F8.4,'     y/t = 0.0')
C
      IF(YF .EQ. -999.0)
     & CALL ASKR(
     &  'Enter flap hinge y location (or 999 to specify y/t)^',YF)
C
      IF(YF .EQ. 999.0) THEN
        CALL ASKR('Enter flap hinge relative y/t location^',YREL)
        YF = TOPY*YREL + BOTY*(1.0-YREL)
      ENDIF
C
      RETURN
      END ! GETXYF



      SUBROUTINE PLOTG
C--------------------------------------------------------------
C     Plots buffer airfoil with ticked chord line or grid
C--------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
      INCLUDE 'XDES.INC'
C
C---- node tick mark size and corner symbol size
      DTICK = GTICK*(SB(NB)-SB(1))
      SSH   = DTICK * 3.0
C
      CALL NCALC(XB,YB,SB,NB,W1,W2)
C
      IF(LGGRID) THEN
        CALL GRDAIR(XGMIN,XGMAX,YGMIN,YGMAX,DXYG,DXYG,CHG,.TRUE.,.TRUE.,
     &              XOFF,XSF,YOFF,YSF, LMASK2)
        XL0 = XMOD(XGMIN)
        YL0 = YMOD(YGMAX) + 2.0*CH
      ELSE
C------ plot chord line and tick marks every 10% chord
        CALL NEWPEN(1)
        CALL PLOT(XMOD(0.0),YMOD(0.0),3)
        CALL PLOT(XMOD(1.0),YMOD(0.0),2)
        DO ITICK=1, 10
          XPLT = FLOAT(ITICK)/10.0
          CALL PLOT(XMOD(XPLT),YMOD(0.003),3)
          CALL PLOT(XMOD(XPLT),YMOD(-.003),2)
        ENDDO
C
        XL0 = XMOD(XBMIN)
        YL0 = YMOD(YBMAX) + 2.0*CH
      ENDIF
      IF(LPLCAM)  YL0 = YSF*(YCMAX-DYOFFC-YOFF) + 2.0*CH
C
      CALL PLFLUSH
C
      CALL NEWPEN(2)
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'black')
C
      IF(LGTICK) THEN
C----- draw tiny tick mark normal to airfoil surface at each panel node
       DO I=2, NB-1
         CALL PLOT(XMOD(XB(I)            ),YMOD(YB(I)            ),3)
         CALL PLOT(XMOD(XB(I)-DTICK*W1(I)),YMOD(YB(I)-DTICK*W2(I)),2)
       ENDDO
      ENDIF
c
cC---- plot symbol at nose
c      CALL NSFIND(STLE,XB,XBP,YB,YBP,SB,NB)
c      XT = SEVAL(STLE,XB,XBP,SB,NB)
c      YT = SEVAL(STLE,YB,YBP,SB,NB)
c      CALL PLSYMB(XMOD(XT),YMOD(YT),0.005*XSF,5,0.0,0)
c
C---- put symbol at any doubled point
      DO I=1, NB-1
        IF(SB(I) .EQ. SB(I+1))
     &     CALL PLSYMB(XMOD(XB(I)),YMOD(YB(I)),SSH,5,0.0,0)
      ENDDO
C
      IF(LPLCAM) THEN
       CALL PLTCAM(' ')
      ENDIF
C
      IF(LGPARM) THEN
       CALL NEWPEN(3)
       CALL GPARPL(XL0,YL0,0.7*CH,.TRUE.,NAME,
     &             CHORDB,AREAB,RADBLE,ANGBTE,
     &             EI11BA,EI22BA,APX1BA,APX2BA,
     &             EI11BT,EI22BT,APX1BT,APX2BT,
     &             THICKB,CAMBRB)
      ENDIF
C
      CALL PLFLUSH
C
      LGEOPL = .TRUE.
      NOVER = 0
C
      RETURN
      END ! PLOTG



      SUBROUTINE PLTCAM(COLIN)
C--------------------------------------------
C     Plots camber & thickness distributions
C--------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) COLIN
      CHARACTER*32 COLC, COLT
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- plot camber/thickness only if camber/tickness plot is being shown
      IF(.NOT.LPLCAM) RETURN
C
      CALL NEWPEN(1)
      CALL GRDAIR(XGMIN,XGMAX,YCMIN,YCMAX,DXYG,DXYG,CHG,.FALSE.,.TRUE.,
     &            XOFF,XSF,DYOFFC+YOFF,YSF, LMASK2)
C     
      CALL GETCAM(XCM,YCM,NCM,XTK,YTK,NTK,
     &            XB,XBP,YB,YBP,SB,NB )
      CALL SCALC(XCM,YCM,SCM,NCM)
      CALL SEGSPL(XCM,XCMP,SCM,NCM)
      CALL SEGSPL(YCM,YCMP,SCM,NCM)
      CALL SCALC(XTK,YTK,STK,NTK)
      CALL SEGSPL(XTK,XTKP,STK,NTK)
      CALL SEGSPL(YTK,YTKP,STK,NTK)
C
      IF(COLIN(1:1) .EQ. ' ') THEN
       COLC = 'green'
       COLT = 'cyan'
      ELSE
       COLC = COLIN
       COLT = COLIN
      ENDIF
C
      CALL NEWPEN(2)
      YOF = YOFF + DYOFFC
      CALL PLTAIR(XTK,XTKP,YTK,YTKP,STK,NTK,XOFF,XSF, YOF, YSF,COLT)
      CALL PLTAIR(XTK,XTKP,YTK,YTKP,STK,NTK,XOFF,XSF,-YOF,-YSF,COLT)
C--- Offset for camber includes offset for LE camber point
      YOFF1C = YOFF + DYOFFC + YCM(1)
      CALL PLTAIR(XCM,XCMP,YCM,YCMP,SCM,NCM,XOFF,XSF, YOFF1C,YSF,COLC)
C
      RETURN
      END ! PLTCAM


      SUBROUTINE PLNEWP(COLOR)
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) COLOR
C
      LOGICAL LCOLOR
      INCLUDE 'XDES.INC'
C
C---- don't plot geometric parameters if camber/tickness plot is being shown
      IF(LPLCAM) RETURN
C
      LCOLOR = COLOR(1:1) .NE. ' '
C
      IF(LCOLOR) THEN
        CALL GETCOLOR(ICOL0)
        CALL NEWCOLORNAME(COLOR)
      ENDIF
C
      CALL NEWPEN(3)
C
      NOVER = NOVER + 1
      IF(LGGRID) THEN
       XL0 = XMOD(XGMIN) +  2.0*CH + 9.0*CH*FLOAT(NOVER)
       YL0 = YMOD(YGMAX) +  2.0*CH
      ELSE
       XL0 = XMOD(XBMIN) +  2.0*CH + 9.0*CH*FLOAT(NOVER)
       YL0 = YMOD(YBMAX) +  2.0*CH
      ENDIF
      
      IF(LPLCAM)  YL0 = YSF*(YCMAX-YOFF-DYOFFC) + 2.0*CH
C
      IF(LGPARM) THEN
        CALL GPARPL(XL0,YL0,0.7*CH,.FALSE.,NAME,
     &              CHORDB,AREAB,RADBLE,ANGBTE,
     &              EI11BA,EI22BA,APX1BA,APX2BA,
     &              EI11BT,EI22BT,APX1BT,APX2BT,
     &              THICKB,CAMBRB)
      ENDIF
C
      IF(LCOLOR) CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      RETURN
      END ! PLNEWP



      SUBROUTINE GPARPL(X0,Y0,CH, LABEL, NAME,
     &                  CHORD,AREA,RADLE,ANGTE,
     &                  EI11A,EI22A,APX1A,APX2A,
     &                  EI11T,EI22T,APX1T,APX2T,
     &                  THICK,CAMBR)
      LOGICAL LABEL
      EXTERNAL PLCHAR
      CHARACTER NAME*(*)
C
      RTD = 45.0/ATAN(1.0)
C
      XSPACE = 30.0*CH
      YSPACE =  2.0*CH
C
      X = X0
      Y = Y0
C
      IF(LABEL) THEN
       CALL PLCHAR(X,Y,CH,'       = ',0.0, 9)
       CALL PLMATH(X,Y,CH,'  Oq     ',0.0, 9)
       CALL PLSUBS(X+3.0*CH,Y,CH,'TE',0.0, 2, PLCHAR)
      ENDIF
      CALL PLNUMB(X+9.0*CH,Y,CH,ANGTE*RTD        ,0.0, 2)
      CALL PLMATH(999.,Y,CH,'"'              ,0.0, 1)
      Y = Y + YSPACE
C
      IF(LABEL) THEN
       CALL PLCHAR(X,Y,CH,'   r   = ',0.0, 9)
       CALL PLSUBS(X+3.0*CH,Y,CH,'LE',0.0, 2, PLCHAR)
      ENDIF
      CALL PLNUMB(X+9.0*CH,Y,CH,RADLE,0.0, 5)
      Y = Y + YSPACE
C
      IF(LABEL) THEN
       CALL PLCHAR(X,Y,CH,'camber = ',0.0, 9)
      ENDIF
      CALL PLNUMB(X+9.0*CH,Y,CH,CAMBR,0.0, 5)
      Y = Y + YSPACE
C
      IF(LABEL) THEN
       CALL PLCHAR(X,Y,CH,'thick. = ',0.0, 9)
      ENDIF
      CALL PLNUMB(X+9.0*CH,Y,CH,THICK,0.0, 5)
      Y = Y + YSPACE
C
      IF(LABEL) THEN
       CALL PLCHAR(X,Y,CH,' area  = ',0.0, 9)
      ENDIF
      CALL PLNUMB(X+9.0*CH,Y,CH, AREA,0.0, 5)
      Y = Y + YSPACE
C
C
c      X = X0  +  XSPACE
c      Y = Y0
cC
c      Y = Y + YSPACE
cC
c      CALL PLMATH(X,Y,1.4*CH,'I',0.0,1)
c      CALL PLMATH(X,Y,CH,'       2     ',0.0,-1)
c      CALL PLCHAR(X,Y,CH,' (y-y ) ds = ',0.0,-1)
c      CALL PLNUMB(999.,Y,CH, 1000.0*EI11T,0.0,4)
c      CALL PLMATH(999.,Y,CH,'#'   ,0.0,1)
c      CALL PLCHAR(999.,Y,CH, '10' ,0.0,2)
c      CALL PLMATH(999.,Y,CH,   '3',0.0,1)
c      CALL PLSUBS(X+4.0*CH,Y,CH,'o',0.0,1,PLCHAR)
c      Y = Y + YSPACE
cC
c      CALL PLMATH(X,Y,1.4*CH,'I',0.0,1)
c      CALL PLMATH(X,Y,CH,'       2     ',0.0,-1)
c      CALL PLCHAR(X,Y,CH,' (y-y ) dA = ',0.0,-1)
c      CALL PLNUMB(999.,Y,CH, 1000.0*EI11A,0.0,4)
c      CALL PLMATH(999.,Y,CH,'#'   ,0.0,1)
c      CALL PLCHAR(999.,Y,CH, '10' ,0.0,2)
c      CALL PLMATH(999.,Y,CH,   '3',0.0,1)
c      CALL PLSUBS(X+4.0*CH,Y,CH,'o',0.0,1,PLCHAR)
c      Y = Y + YSPACE
cC
c      CALL PLMATH(X,Y,CH,'             ',0.0,-1)
c      CALL PLCHAR(X,Y,CH,'      area = ',0.0,-1)
c      CALL PLNUMB(999.,Y,CH, AREA,0.0, 5)
c      Y = Y + YSPACE
C
C--- Plot airfoil name over data list
      CALL PLCHAR(X+9.0*CH,Y,CH,NAME,0.0, 12)
C
      RETURN
      END ! GPARPL



      SUBROUTINE GRDAIR(XGMIN,XGMAX, YGMIN,YGMAX,DXGN,DYGN,CHG,
     &                  LXAXIS,LYAXIS,
     &                  XOFF,XSF,YOFF,YSF, LMASK)
      LOGICAL LXAXIS,LYAXIS
C----------------------------------------
C     Plots grid with axes.
C     Intended for airfoil plot.
C----------------------------------------
      INCLUDE 'XDES.INC'
C
      CALL NEWPEN(1)
C
C---- plot outline
      CALL PLOT(XMOD(XGMIN),YMOD(YGMIN),3)
      CALL PLOT(XMOD(XGMAX),YMOD(YGMIN),2)
      CALL PLOT(XMOD(XGMAX),YMOD(YGMAX),2)
      CALL PLOT(XMOD(XGMIN),YMOD(YGMAX),2)
      CALL PLOT(XMOD(XGMIN),YMOD(YGMIN),2)
C
      IF(LXAXIS)
     &  CALL XAXIS(XMOD(XGMIN),YMOD(YGMIN),(XGMAX-XGMIN)*XSF,
     &             DXGN*XSF, XGMIN,DXGN,CHG,-2)
      IF(LYAXIS)
     &  CALL YAXIS(XMOD(XGMIN),YMOD(YGMIN),(YGMAX-YGMIN)*YSF,
     &             DYGN*YSF, YGMIN,DYGN,CHG,-2)
C
C---- fine grid
      NXG = INT((XGMAX-XGMIN)/DXGN + 0.1)
      NYG = INT((YGMAX-YGMIN)/DYGN + 0.1)
      NXG = MAX(1,NXG)
      NYG = MAX(1,NYG)
C
      X0 = XMOD(XGMIN)
      Y0 = YMOD(YGMIN)
      DXG = (XMOD(XGMAX)-X0)/NXG
      DYG = (YMOD(YGMAX)-Y0)/NYG
      CALL PLGRID(X0,Y0,NXG,DXG,NYG,DYG, LMASK)
C
      RETURN
      END ! GRDAIR



      SUBROUTINE PLTAIR(XX,XXP,YY,YYP,SS,NN, XOFF,XSF,YOFF,YSF,COLOR)
      DIMENSION XX(NN), XXP(NN), YY(NN), YYP(NN), SS(NN)
      CHARACTER*(*) COLOR
C-----------------------------
C     Plots passed-in airfoil
C-----------------------------
      LOGICAL LCOLOR
      XMOD(XTMP) = XSF * (XTMP - XOFF)
      YMOD(YTMP) = YSF * (YTMP - YOFF)
C
      NT = 20
ccc      NT = 50
C
      LCOLOR = COLOR(1:1) .NE. ' '
C
      IF(LCOLOR) THEN
        CALL GETCOLOR(ICOL0)
        CALL NEWCOLORNAME(COLOR)
      ENDIF
C
      DO 60 I=2, NN
        DS = SS(I) - SS(I-1)
        CALL PLOT(XMOD(XX(I-1)),YMOD(YY(I-1)),3)
C
C------ subdivide current panel into NT segments for smoother airfoil plot
        DO 610 IT=1, NT
          ST = SS(I-1) + DS*FLOAT(IT)/FLOAT(NT)
          XT = SEVAL(ST,XX,XXP,SS,NN)
          YT = SEVAL(ST,YY,YYP,SS,NN)
          CALL PLOT(XMOD(XT),YMOD(YT),2)
  610   CONTINUE
   60 CONTINUE
C
      IF(LCOLOR) CALL NEWCOLOR(ICOL0)
C
      CALL PLFLUSH
C
      RETURN
      END ! PLTAIR



      SUBROUTINE OVER(FNAME1)
C----------------------------------------------------
C     Overlays plot of airfoil from coordinate file.
C----------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*(*) FNAME1
C
      CHARACTER*32 NAME0, NAMEW
      CHARACTER*80 ISPARS0
C
      IF(FNAME1(1:1).NE.' ') THEN
       FNAME = FNAME1
      ELSE
C----- no argument... get it somehow
       IF(ONAME(1:1).NE.' ') THEN
C------ offer existing default
        WRITE(*,1100) ONAME
 1100   FORMAT(/' Enter filename:  ', A)
        READ(*,1000) FNAME
 1000   FORMAT(A)
        CALL STRIP(FNAME,NFN)
        IF(NFN.EQ.0) FNAME = ONAME
       ELSE
C------ just ask for filename
        CALL ASKS('Enter filename^',FNAME)
       ENDIF
      ENDIF
C
      LU = 9
      CALL AREAD(LU,FNAME,2*IQX,W1,W2,NN,NAME0,ISPARS0,ITYPE,1)
      IF(ITYPE.EQ.0) RETURN
C
C---- set new default filename
      ONAME = FNAME
C
      IF(LNORM) THEN
C----- normalize to unit chord
       CALL NORM(W1,W3,W2,W4,W5,NN)
      ELSE
       CALL SCALC(W1,W2,W5,NN)
       CALL SEGSPL(W1,W3,W5,NN)
       CALL SEGSPL(W2,W4,W5,NN)
      ENDIF
C
      NAMEW  = NAME
      SWLE   = SBLE  
      CHORDW = CHORDB
      AREAW  = AREAB 
      RADWLE = RADBLE
      ANGWTE = ANGBTE
      EI11WA = EI11BA
      EI22WA = EI22BA
      APX1WA = APX1BA
      APX2WA = APX2BA
      EI11WT = EI11BT
      EI22WT = EI22BT
      APX1WT = APX1BT
      APX2WT = APX2BT
      THICKW = THICKB
      CAMBRW = CAMBRB
C
      NAME = NAME0
      CALL GEOPAR(W1,W3,W2,W4,W5,NN,W6,
     &            SBLE,CHORDB,AREAB,RADBLE,ANGBTE,
     &            EI11BA,EI22BA,APX1BA,APX2BA,
     &            EI11BT,EI22BT,APX1BT,APX2BT,
     &            THICKB,CAMBRB )
C
      IF(.NOT.LPLOT) THEN
       CALL PLTINI
ccc       CALL PLOT(0.05,0.30,-3)
      ENDIF
C
C
      CALL GETCOLOR(ICOL0)
      ICOL = 3 + MOD(NOVER,6)
      IF(ICOL .GE. 5) ICOL = ICOL + 1
      CALL NEWPEN(2)
      CALL NEWCOLOR(ICOL)
C
      CALL PLTAIR(W1,W3,W2,W4,W5,NN, XOFF,XSF, YOFF,YSF,' ')
      CALL PLNEWP(' ')
C
      CALL NEWCOLOR(ICOL0)
C
C
C---- restore parameters
      NAME   = NAMEW
      SBLE   = SWLE  
      CHORDB = CHORDW
      AREAB  = AREAW 
      RADBLE = RADWLE
      ANGBTE = ANGWTE
      EI11BA = EI11WA
      EI22BA = EI22WA
      APX1BA = APX1WA
      APX2BA = APX2WA
      EI11BT = EI11WT
      EI22BT = EI22WT
      APX1BT = APX1WT
      APX2BT = APX2WT
      THICKB = THICKW
      CAMBRB = CAMBRW
C
      RETURN
      END ! OVER


