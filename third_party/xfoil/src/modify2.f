C***********************************************************************
C    Module:  modify.f
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

      SUBROUTINE MODIFY(IX,IFRST,ILAST,NSIDE,NLINE,
     &                  X,Y,YD, LBLEND,
     &                  IMOD1,IMOD2,ISMOD,ILMOD,
     &                  XMOD,YMOD, XWIN,YWIN, SIZE,
     &                  XOFF,YOFF,XSF,YSF, COLPNT,COLMOD,
     &                  NEWPLOT )
      DIMENSION IFRST(NSIDE), ILAST(NSIDE)
      DIMENSION X(IX), Y(IX,NLINE), YD(IX,NLINE)
      DIMENSION XMOD(2),YMOD(2), XWIN(2),YWIN(2)
      LOGICAL LBLEND
      CHARACTER*(*) COLPNT, COLMOD
      EXTERNAL NEWPLOT
C--------------------------------------------------------------------------
C     Allows user to modify functions Y1(X),Y2(X)... via cursor input.
C
C     Cursor-specified Xu,Yu values are sorted by Xu and splined.
C     The resulting spline function Yu(X) is interrogated at input 
C     X(i) points to obtain the modified Y(i,L) values.
C
C  Input:   IX          first dimension of X,Y arrays
C           IFRST(s)    first i index in segment s
C           ILAST(s)    last  i index in segment s
C           NSIDE       number of X segments :   s = 1..NSIDE
C           NLINE       number of Y functions:   l = 1..NLINE
C           X(i)        X values
C           Y(i,l)      Y values
C           YD(i,l)     spline derivative array dY/dX (used only if LSLOPE=T)
C           LBLEND      if T, blends input Yu(Xu) with Y(X) at input endpoints
C           XMOD(2)     x-limits of box for cursor input
C           YMOD(2)     y-limits of box for cursor input
C           XWIN(2)     x-limits of plot window
C           YWIN(2)     y-limits of plot window
C           SIZE        overall object scaling size
C           XOFF        plot offsets,scales used to plot X(S),Y(S)
C           YOFF         "  Xplot = (X-XOFF)*XSF
C           XSF          "  Yplot = (Y-YOFF)*YSF
C           YSF          "
C           COLPNT      color of symbols at cursor-selected points
C           COLMOD      if not blank, plot modified Y(i,l) with color COLMOD
C           NEWPLOT     subroutine to be called for refreshed plot
C
C  Output:  Y(i,l)      modified Y values
C           IMOD1       first i index of modified Y(i,l) values
C           IMOD2       last  i index of modified Y(i,l) values
C           ISMOD       index s of segment containing IMOD1,IMOD2
C           ILMOD       index l of Y(i,l) function which was modified
C--------------------------------------------------------------------------
C
C---- local arrays for accumulating user-specified points
      PARAMETER (NUX=100)
      DIMENSION XU(NUX), YU(NUX)
      DIMENSION XP(NUX), YP(NUX), YPD(NUX)
      DIMENSION IPSORT(NUX)
      LOGICAL LDONE, LPLNEW, LEDIT, LEXIT
C
      LOGICAL LGUI
      CHARACTER*1 CHKEY
C
      DATA SH /0.010/
C
      CALL GETCOLOR(ICOL0)
      CALL GETPEN(IPEN0)
C
      KDONE  = 1
      KABORT = 2
      KERASE = 3
      KSWITCH = 4
      KINSIDE = 5
C
      XDWIN = XWIN(2) - XWIN(1)
      YDWIN = YWIN(2) - YWIN(1)
C
      XWS = XDWIN/SIZE
      YWS = YDWIN/SIZE
C---- initialize current plot scales,offsets in case KEYOFF changes them
      XSF0 = XSF
      YSF0 = YSF
      XOFF0 = XOFF
      YOFF0 = YOFF
C
      WRITE(*,*)
      WRITE(*,*) '_________________ Spline input mode _________________'
      WRITE(*,*) ' Click on new values to input new shape, or ...'
      WRITE(*,*) ' ... type S,E,A,D or click buttons for special action'
      WRITE(*,*) ' ... type I,O,P to In,Out,Pan towards cursor'
      WRITE(*,*)
C
      NU = 0
      NP = 0
      JP = 0
      IMOD1 = 0
      IMOD2 = 0
      LPLNEW = .FALSE.
      LEDIT  = .FALSE.
      LEXIT  = .FALSE.

 1100 FORMAT(1X, I3)
C
C=========================================================================
C---- Plot everything (background plot, user points, etc)
C     XU,YU (NU) are user specfied points in plot coordinates
C     X,Y contains input X,Y arrays for modification
C
 5    CONTINUE
cc      WRITE(*,*) 'MODIFY - Replotting everything LPLNEW ',LPLNEW
cc      WRITE(*,*) '    NU,IMOD1,IMOD2 ',NU,IMOD1,IMOD2
C
      IF(LPLNEW) THEN
C------- scales,offsets have changed... replot
       CALL NEWCOLOR(ICOL0)
       CALL NEWPLOT
C
       CALL NEWCOLORNAME(COLPNT)
C------- plot current store of input points
       DO IU = 1, NU
         CALL PLSYMB(XU(IU),YU(IU),SH,3,0.0,0)
       ENDDO
       LPLNEW = .FALSE.
      ENDIF
C
C------ plot splined user changes in modified region
      IF(NP.GT.0) THEN
       IF(COLMOD(1:1).NE.' ') THEN
        CALL NEWCOLORNAME(COLMOD)
        IPEN = 3
        XX = (XP(1)-XOFF)*XSF
        YY = (YP(1)-YOFF)*YSF
        CALL PLOT(XX,YY,IPEN)
        NPL = 11
        DO I = 2, NP
          DX = XP(I)-XP(I-1)
          DO L = 1, NPL
            FRAC = FLOAT(L-1)/FLOAT(NPL-1)          
            XX = XP(I-1) + FRAC*DX
            YY = SEVAL(XX,YP,YPD,XP,NP)
            XX = (XX-XOFF)*XSF
            YY = (YY-YOFF)*YSF
            CALL PLOT(XX,YY,IPEN)
            IPEN = 2
          ENDDO
        ENDDO
        CALL PLFLUSH
       ENDIF
      ENDIF
C
C---- put up GUI boxes
      CALL NEWPEN(5)
      Y1 = YWIN(1) + 0.01*YDWIN
      Y2 = YWIN(1) + 0.05*YDWIN
C
      X1 = XWIN(2) - 0.08*XDWIN
      X2 = XWIN(2) - 0.01*XDWIN
      CALL GUIBOX(KDONE , X1,X2,Y1,Y2, 'GREEN', ' Done ')
C
      X1 = X1 - 0.08*XDWIN
      X2 = X2 - 0.08*XDWIN
      CALL GUIBOX(KABORT, X1,X2,Y1,Y2, 'RED'   , ' Abort ')
C
      IF(.NOT.LEDIT) THEN
       X1 = X1 - 0.08*XDWIN
       X2 = X2 - 0.08*XDWIN
       CALL GUIBOX(KERASE, X1,X2,Y1,Y2, 'YELLOW', ' Erase ')

       X1 = XWIN(2) - 0.46*XDWIN
       X2 = XWIN(2) - 0.26*XDWIN
       CALL GUIBOX(KSWITCH, X1,X2,Y1,Y2, 'BLUE',' Switch to edit mode ')
      ENDIF
C
      X1 = XMOD(1)
      X2 = XMOD(2)
      Y1 = YMOD(1)
      Y2 = YMOD(2)
      CALL GUIBOX(KINSIDE, X1,X2,Y1,Y2, 'ORANGE' , ' ' )
      CALL PLFLUSH
C
      CALL NEWPEN(IPEN0)
C
      XWS = XDWIN/SIZE
      YWS = YDWIN/SIZE
C
C========================================================================
 10   CONTINUE
      CALL NEWCOLORNAME(COLPNT)
ccc   WRITE(*,*) 'at label 10 LEDIT, JP ',LEDIT, JP
C
C------ interactive dragging of points
C------ if in edit mode check for edit/drag cursor GUI actions
      IF(LEDIT .AND. JP.NE.0) THEN
       CALL GETCURSORXYC(XIN,YIN,IBTN)
ccc     WRITE(*,*) XIN,YIN,IBTN
       IF(IBTN.EQ.0) THEN
cc      WRITE(*,*) 'IBTN ',IBTN
        JP = 0
        GO TO 20
       ENDIF
C------ modify point plot coordinates
       XU(JP) = XIN
       YU(JP) = YIN
C------ reprocess spline
       GO TO 30
      ENDIF
C
C
C========================================================================
C------ fetch x-y point coordinates from user mouse button or keypress
  20  CALL GETCURSORXY(XIN,YIN,CHKEY)
ccc      WRITE(*,*) 'at label 20 XIN,YIN,CHKEY ',XIN,YIN,CHKEY
C------ save current plot scales,offsets in case KEYOFF changes them
        XSF0 = XSF
        YSF0 = YSF
        XOFF0 = XOFF
        YOFF0 = YOFF
C
C------ check for pan,zoom operations based on character input CHKEY
        CALL KEYOFF(XIN,YIN,CHKEY, 
     &              XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C------- adjust points for new plot offsets and scales 
        IF(LPLNEW) THEN
          DO IU = 1, NU
            XU(IU) = ((XU(IU)/XSF0 + XOFF0) - XOFF)*XSF
            YU(IU) = ((YU(IU)/YSF0 + YOFF0) - YOFF)*YSF
          ENDDO
C------- replot everything
          GO TO 5
        ENDIF
C
C------ check for GUI actions
C
C-------------------------------------------------------
C------ ABORT quits input and exits without changes
        IF(LGUI(KABORT,XIN,YIN)
     &     .OR. INDEX('Aa',CHKEY).GT.0) THEN
         WRITE(*,*) 'ABORT received'
C------- return with no changes
         GO TO 90
C
C-------------------------------------------------------
C------ DONE with modifications
        ELSEIF(LGUI(KDONE,XIN,YIN)
     &         .OR. INDEX('Dd',CHKEY).GT.0) THEN
         WRITE(*,*) 'DONE received'
         LEXIT = .TRUE.
C------- go process inputs
         GO TO 30
C
C-------------------------------------------------------
C------ ERASE last point
        ELSEIF(.NOT. LEDIT .AND. 
     &         (LGUI(KERASE,XIN,YIN) .OR. INDEX('Ee',CHKEY).GT.0)) THEN
         IF(NU.LE.0) THEN
          WRITE(*,*) 'No more points to clear'
          NU = 0
         ELSE
C-------- clear previous point, overplot it white to clear it from screen
          CALL NEWCOLORNAME('WHITE')
          CALL PLSYMB(XU(NU),YU(NU),SH,3,0.0,0)
          CALL PLFLUSH
          NU = NU - 1
          WRITE(*,1100) NU
          GO TO 10
         ENDIF
C
C-------------------------------------------------------
C------ SWITCH to edit mode, drag, delete or add points 
        ELSEIF(LGUI(KSWITCH,XIN,YIN)
     &         .OR. INDEX('Ss',CHKEY).GT.0) THEN
C------- switch from point input to editing mode
         LEDIT = .NOT. LEDIT
         WRITE(*,*)
         WRITE(*,*) '________________ Spline edit mode ________________'
         WRITE(*,*) ' Drag points to change shape, or ...'
         WRITE(*,*) ' ... type A,D or click buttons  for special action'
         WRITE(*,*) ' ... type N to insert new point at cursor'
         WRITE(*,*) ' ... type E to erase existing point nearest cursor'
         WRITE(*,*) ' ... type I,O,P to In,Out,Pan towards cursor'
         GO TO 30
C
C-------------------------------------------------------
C------ Check for input point clicked in window
        ELSEIF(LGUI(KINSIDE,XIN,YIN)) THEN
C------- normal click inside modify-window gets treated differently for editing
C  
C------------------------------------------------
         IF(.NOT. LEDIT) THEN
cc         WRITE(*,*) 'Inside point non-edit mode '
           IF(NU.GE.NUX) THEN
            WRITE(*,*) 'MODIFY: User-input array limit NUX reached'
            GO TO 90
           ENDIF
C------- accumulation of points, add point to list
           NU = NU + 1
           XU(NU) = XIN
           YU(NU) = YIN
C------- plot small cross at input point
           CALL PLSYMB(XU(NU),YU(NU),SH,3,0.0,0)
           CALL PLFLUSH
           WRITE(*,1100) NU
C
           GO TO 20
C
C------------------------------------------------
         ELSE
C--------- edit mode
cc         WRITE(*,*) 'Inside point edit mode '
C
C--------- find closest point to cursor     
           JP = 1
           DSQMIN = (XIN-XU(JP))**2 + (YIN-YU(JP))**2
           DO J = 2, NU
             DSQ = (XIN-XU(J))**2 + (YIN-YU(J))**2
             IF(DSQ.LT.DSQMIN) THEN
               DSQMIN = DSQ
               JP = J
             ENDIF
           END DO
ccc        WRITE(*,*) 'Closest point JP ',JP,'Dsq ',DSQMIN
C
C--------- Input key options: N (new point), E (erase point)
           IF(INDEX('Nn',CHKEY).GT.0) THEN
C---------- add point to arrays at cursor position
            WRITE(*,*) 'Adding point at cursor...'
C---------- find position in x
            JA = 0
            DO J = 2,NU
             IF(XIN.GE.XU(J-1) .AND. XIN.LT.XU(J)) JA = J
            END DO         
            IF(XIN.LT.XU(1 )) JA = 1
            IF(XIN.GT.XU(NU)) JA = NU+1
C
            IF(NU.GE.NUX) THEN
             WRITE(*,*) 'MODIXY: User-input array limit NUX reached'
             GO TO 90
            ENDIF
C
C---------- shift array and add point
            IF(JA.NE.0) THEN
             DO J = NU, JA, -1
               XU(J+1) = XU(J)
               YU(J+1) = YU(J)
             END DO
             XU(JA) = XIN
             YU(JA) = YIN
             NU = NU+1
C----------- plot small cross at input point
             CALL PLSYMB(XU(NU),YU(NU),SH,3,0.0,0)
             CALL PLFLUSH
             WRITE(*,1100) NU
            ENDIF
            JP = 0
            GO TO 30
           ENDIF
C
C--------- delete point at cursor position
           IF(CHKEY.EQ.'E' .OR. CHKEY.EQ.'e') THEN
            WRITE(*,2050) JP
 2050       FORMAT(1X,'Deleting point',I3,' nearest cursor')
            DO J = JP+1,NU
              XU(J-1) = XU(J)
              YU(J-1) = YU(J)
            END DO
            NU = NU-1
            WRITE(*,1100) NU
C
            JP = 0
            LPLNEW = .TRUE.
            GO TO 30
           ENDIF
C
         ENDIF
C
        ELSE
C------- click outside of plot window
         WRITE(*,*) 'Click outside plot window ignored'
         GO TO 10
C
        ENDIF
C
C=======================================================================
C---- Process points input by user to define splined changes
 30   CONTINUE
C
      IF(NU.LT.2) THEN
       WRITE(*,*)
       WRITE(*,*) 'Need at least 2 points'
       GO TO 90
      ENDIF
C
C---- put user-specified points into X,Y coordinate using plot offsets, scales
      DO IU = 1, NU
        XP(IU) = XU(IU)/XSF + XOFF
        YP(IU) = YU(IU)/YSF + YOFF
      ENDDO
      NP = NU
C---- set first-specified point
      XPSP1 = XP(1)
      YPSP1 = YP(1)
C
C---- sort XP,YP points in XP (use spline array YPD as temporary storage)
      CALL HSORT(NP,XP,IPSORT)
C
      DO KSORT = 1, NP
        IP = IPSORT(KSORT)
        YPD(KSORT) = XP(IP)
      ENDDO
      DO IP = 1, NP
        XP(IP) = YPD(IP)
      ENDDO
C
      DO KSORT = 1, NP
        IP = IPSORT(KSORT)
        YPD(KSORT) = YP(IP)
      ENDDO
      DO IP = 1, NP
        YP(IP) = YPD(IP)
      ENDDO
C
C---- remove doubled endpoints and tripled interior points
      DO IPASS = 1, 12345
        LDONE = .TRUE.
        IP = 2
        IF(XP(IP).EQ.XP(IP-1)) THEN
         LDONE = .FALSE.
         IPREM = IP
        ENDIF
        DO IP = 3, NP
          IF( XP(IP).EQ.XP(IP-1) .AND.
     &        XP(IP).EQ.XP(IP-2)      ) THEN
           LDONE = .FALSE.
           IPREM = IP
          ENDIF
        ENDDO
        IP = NP
        IF(XP(IP).EQ.XP(IP-1)) THEN
         LDONE = .FALSE.
         IPREM = IP
        ENDIF
C
        IF(LDONE) THEN
         GO TO 40
        ELSE
         DO IP = IPREM, NP-1
           XP(IP) = XP(IP+1)
           YP(IP) = YP(IP+1)
         ENDDO
         NP = NP - 1
        ENDIF
      ENDDO
C
C---- pick up here when no more points to be removed
 40   CONTINUE
      IF(NP.LT.2) THEN
       WRITE(*,*)
       WRITE(*,*) 'Need at least 2 points'
       GO TO 90
      ENDIF
C
C---- find which X,Y input point is closest to first-specified point
      ISMOD = 1
      ILMOD = 1
C---- go over all surface points
      DSQMIN = 1.0E24
      DO IL = 1, NLINE
        DO IS = 1, NSIDE
          DO I = IFRST(IS), ILAST(IS)
            XPI = X(I   )
            YPI = Y(I,IL)
            DSQ = (XPI-XPSP1)**2 + (YPI-YPSP1)**2
            IF(DSQ .LT. DSQMIN) THEN
C------------ this point is the closest so far... note its indices
              DSQMIN = DSQ
              ISMOD = IS
              ILMOD = IL
            ENDIF
          ENDDO
        ENDDO
      ENDDO
C
C---- set side and function to be modified
      IS = ISMOD
      IL = ILMOD
C
      IF(LBLEND) THEN
C----- reset Y and dY/dX at first and last points of modified interval
       X1 = X(IFRST(IS))
       X2 = X(ILAST(IS))
       I = IFRST(IS)
       N = ILAST(IS) - IFRST(IS) + 1
C
       IP = 1
       IF(XP(IP).GE.X1 .AND. XP(IP).LE.X2) THEN
C------ set function and derivative at left endpoint
        YP(IP) = SEVAL(XP(IP),Y(I,IL),YD(I,IL),X(I),N)
        YD1    = DEVAL(XP(IP),Y(I,IL),YD(I,IL),X(I),N)
       ELSE
        YD1 = -999.0
       ENDIF
C
       IP = NP
       IF(XP(IP).GE.X1 .AND. XP(IP).LE.X2) THEN
        YP(IP) = SEVAL(XP(IP),Y(I,IL),YD(I,IL),X(I),N)
        YD2    = DEVAL(XP(IP),Y(I,IL),YD(I,IL),X(I),N)
       ELSE
        YD2 = -999.0
       ENDIF
C
      ELSE
C----- use natural spline end conditions (zero 3rd derivative)
       YD1 = -999.0
       YD2 = -999.0
      ENDIF
C
C---- spline input function values
      CALL SEGSPLD(YP,YPD,XP,NP,YD1,YD2)
C
C---- replot screen with splined line and points
      LPLNEW = .TRUE.
      IF(.NOT.LEXIT) GO TO 5
C
C
C---- Modify input points in user-specified segment
C---- go over all points on modified segment
 60   IMOD1 = IFRST(IS)
      DO I = IFRST(IS), ILAST(IS)
        XI = X(I)
        IF    (XI .LT. XP( 1)) THEN
C------- current point is before modified interval...try next point
         IMOD1 = I
        ELSEIF(XI .LE. XP(NP)) THEN
C------- stuff new point into Vspec array and plot it
         Y(I,IL) = SEVAL(XI,YP,YPD,XP,NP)
        ELSE
C------- went past modified interval...finish up
         IMOD2 = I
         GO TO 50
        ENDIF
      ENDDO
      IMOD2 = ILAST(IS)
C
C---- Replot with changes
 50   CALL NEWCOLOR(ICOL0)
      CALL NEWPLOT
C------- plot splined input line in modified region
      IF(IMOD1.NE.0 .AND. IMOD2.NE.0) THEN
       IF(COLMOD(1:1).NE.' ') THEN
C----- plot modified function over modified interval
        CALL NEWCOLORNAME(COLMOD)
        IPEN = 3
        DO I = IMOD1, IMOD2
C----- scale X,Y to plot coordinates
          XP = (X(I   )-XOFF)*XSF
          YP = (Y(I,IL)-YOFF)*YSF
          CALL PLOT(XP,YP,IPEN)
          IPEN = 2
        ENDDO
        CALL PLFLUSH
       ENDIF
      ENDIF
C
      CALL NEWCOLOR(ICOL0)
      RETURN
C
C-------------------------------------------------
 90   CONTINUE
      WRITE(*,*) 'No changes made'
      IMOD1 = IFRST(1)
      IMOD2 = IFRST(1) - 1
      ISMOD = 1
      ILMOD = 1
      CALL NEWCOLOR(ICOL0)
      RETURN
C
      END ! MODIFY



      SUBROUTINE MODIXY(IX,IFRST,ILAST,NSIDE,
     &                  X,Y,XD,YD,S, LBLEND,
     &                  IMOD1,IMOD2,ISMOD,
     &                  XMOD,YMOD, XWIN,YWIN,SIZE,
     &                  XOFF,YOFF,XSF,YSF, LMODPL,
     &                  NEWPLOT )
      DIMENSION IFRST(NSIDE), ILAST(NSIDE)
      DIMENSION X(IX),Y(IX), XD(IX),YD(IX), S(IX)
      DIMENSION XMOD(2),YMOD(2), XWIN(2),YWIN(2)
      LOGICAL LBLEND, LMODPL
      EXTERNAL NEWPLOT
C--------------------------------------------------------------------------
C     Allows user to modify contours X(S),Y(S) via cursor input.
C
C     Cursor-specified Xu,Yu values are splined in Su.
C     The resulting spline functions Xu(Su),Yu(Su) are interrogated 
C     at input S(i) points to obtain the modified X(i),Y(i) values.
C
C  Input:   IX          first dimension of X,Y arrays
C           IFRST(s)    first i index in segment s
C           ILAST(s)    last  i index in segment s
C           NSIDE       number of X segments :   s = 1..NSIDE
C           X(i)        X values
C           Y(i)        Y values
C           XD(i)       spline derivative array dX/dS (used only if LSLOPE=T)
C           YD(i)       spline derivative array dY/dS (used only if LSLOPE=T)
C           S(i)        S values
C           LBLEND      if T, blends input Yu(Xu) with Y(X) at input endpoints
C           XMOD(2)     x-limits of box for cursor input
C           YMOD(2)     y-limits of box for cursor input
C           XWIN(2)     x-limits of plot window
C           YWIN(2)     y-limits of plot window
C           SIZE        overall object scaling size
C           XOFF        plot offsets,scales used to plot X(S),Y(S)
C           YOFF         "  Xplot = (X-XOFF)*XSF
C           XSF          "  Yplot = (Y-YOFF)*YSF
C           YSF          "
C           LMODPL      if T, plot modified X(i),Y(i) points
C           NEWPLOT     subroutine to be called for refreshed plot
C
C  Output:  X(i)        modified X values
C           Y(i)        modified Y values
C           IMOD1       first i index of modified X(i),Y(i) values
C           IMOD2       last  i index of modified X(i),Y(i) values
C           ISMOD       index s of segment containing IMOD1,IMOD2
C--------------------------------------------------------------------------
C
C---- local arrays for accumulating user-specified points
      PARAMETER (NUX=200)
      DIMENSION XU(NUX), YU(NUX), XUD(NUX), YUD(NUX), SU(NUX)
      LOGICAL LDONE, LPLNEW
C
      LOGICAL LGUI
      CHARACTER*1 CHKEY
C
      DATA SH /0.010/
C
      CALL GETCOLOR(ICOL0)
      CALL GETPEN(IPEN0)
C
      KDONE  = 1
      KERASE = 2
      KABORT = 3
      KINSIDE = 4
C
      XDWIN = XWIN(2) - XWIN(1)
      YDWIN = YWIN(2) - YWIN(1)
C
      XWS = XDWIN/SIZE
      YWS = YDWIN/SIZE
C
      WRITE(*,*)
      WRITE(*,*) 'Click on new values to change shape...'
      WRITE(*,*) 'Or.. Click buttons or type A,E,D for special action'
      WRITE(*,*) 'Or.. Type I,O,P to In,Out,Pan with cursor...'
      WRITE(*,*)
C
      NUBEG = 1
C
 5    CONTINUE
      CALL NEWPEN(5)
C
      X1 = XWIN(1) + 0.71*XDWIN
      X2 = XWIN(1) + 0.79*XDWIN
      Y1 = YWIN(1) + 0.01*YDWIN
      Y2 = YWIN(1) + 0.05*YDWIN
      CALL GUIBOX(KABORT, X1,X2,Y1,Y2, 'RED'   , ' Abort ')
C
      X1 = XWIN(1) + 0.81*XDWIN
      X2 = XWIN(1) + 0.89*XDWIN
      Y1 = YWIN(1) + 0.01*YDWIN
      Y2 = YWIN(1) + 0.05*YDWIN
      CALL GUIBOX(KERASE, X1,X2,Y1,Y2, 'YELLOW', ' Erase ')
C
      X1 = XWIN(1) + 0.91*XDWIN
      X2 = XWIN(1) + 0.99*XDWIN
      Y1 = YWIN(1) + 0.01*YDWIN
      Y2 = YWIN(1) + 0.05*YDWIN
      CALL GUIBOX(KDONE , X1,X2,Y1,Y2, 'GREEN', ' Done ')
C
      X1 = XMOD(1)
      X2 = XMOD(2)
      Y1 = YMOD(1)
      Y2 = YMOD(2)
      CALL GUIBOX(KINSIDE, X1,X2,Y1,Y2, 'ORANGE' , ' ' )
C
      CALL PLFLUSH
C
      CALL NEWPEN(IPEN0)
C
C
 10   CONTINUE
      CALL NEWCOLORNAME('MAGENTA')
      DO NU = NUBEG, NUX
C
C------ fetch x-y point coordinates from user
        CALL GETCURSORXY(XU(NU),YU(NU),CHKEY)
CCC        write(*,*) ichar(chkey)
C
C------ save current plot scales,offsets in case KEYOFF changes them
        XSF0 = XSF
        YSF0 = YSF
        XOFF0 = XOFF
        YOFF0 = YOFF
C
C------ do possible pan,zoom operations based on CHKEY
        CALL KEYOFF(XU(NU),YU(NU),CHKEY, 
     &              XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
        IF(LPLNEW) THEN
C------- scales,offsets have changed... replot
         CALL NEWCOLOR(ICOL0)
         CALL NEWPLOT
C
         CALL NEWCOLORNAME('MAGENTA')
C
C------- adjust for new plot offsets and scales, replot current store of clicks
         DO IU = 1, NU-1
           XU(IU) = ((XU(IU)/XSF0 + XOFF0) - XOFF)*XSF
           YU(IU) = ((YU(IU)/YSF0 + YOFF0) - YOFF)*YSF
           CALL PLSYMB(XU(IU),YU(IU),SH,3,0.0,0)
         ENDDO
C
C------- will start by fetching NUBEG'th click point
         NUBEG = NU
         GO TO 5
        ENDIF
C
C
C------ process special-action button keys
        IF    (LGUI(KABORT,XU(NU),YU(NU))
     &         .OR. INDEX('Aa',CHKEY).GT.0) THEN
C------- return with no changes
         GO TO 90
C
        ELSEIF(LGUI(KERASE,XU(NU),YU(NU))
     &         .OR. INDEX('Ee',CHKEY).GT.0) THEN
         IF(NU.LE.1) THEN
          WRITE(*,*) 'No more points to clear'
          NUBEG = 1
         ELSE
C-------- clear previous point, overplot it white to clear it from screen
          NUBEG = NU - 1
          CALL NEWCOLORNAME('WHITE')
          CALL PLSYMB(XU(NUBEG),YU(NUBEG),SH,3,0.0,0)
          CALL PLFLUSH
         ENDIF
C
         WRITE(*,1100) NUBEG-1
C
C------- keep accepting points starting from NUBEG
         GO TO 10
C
        ELSEIF(LGUI(KDONE,XU(NU),YU(NU))
     &         .OR. INDEX('Dd',CHKEY).GT.0) THEN
C------- go process inputs
         GO TO 20
C
        ELSEIF(LGUI(KINSIDE,XU(NU),YU(NU))) THEN
C------- normal click inside modify-window: plot small cross at input point
         CALL PLSYMB(XU(NU),YU(NU),SH,3,0.0,0)
         CALL PLFLUSH
C
        ELSE
C------- must be somewhere outside
         GO TO 20
C
        ENDIF
C
        WRITE(*,1100) NU
 1100   FORMAT(1X, I3)
C        
      ENDDO
      WRITE(*,*) 'MODIXY: User-input array limit NUX reached'
C
C---- Process user points into spline matched at ends to original data
 20   CONTINUE
cc      IF(INDEX('Dd',CHKEY).GT.0) THEN
ccC----- last point was entered with a "D" ...  add it to list
cc       CALL PLSYMB(XU(NU),YU(NU),SH,3,0.0,0)
cc       CALL PLFLUSH
cc      ELSE
C----- discard last point
       NU = NU-1
cc      ENDIF
C
C
      IF(NU.LT.2) THEN
       WRITE(*,*)
       WRITE(*,*) 'Need at least 2 points'
       GO TO 90
      ENDIF
C
C---- set first- and last-specified point
      XUSP1 = XU(1)
      YUSP1 = YU(1)
C
      XUSP2 = XU(NU)
      YUSP2 = YU(NU)
C
C---- undo plot offsets and scales
      DO IU = 1, NU
        XU(IU) = XU(IU)/XSF + XOFF
        YU(IU) = YU(IU)/YSF + YOFF
      ENDDO
C
C---- remove doubled endpoints and tripled interior points
      DO IPASS = 1, 12345
        LDONE = .TRUE.
        IU = 2
        IF(XU(IU).EQ.XU(IU-1)) THEN
         LDONE = .FALSE.
         IUREM = IU
        ENDIF
        DO IU = 3, NU
          IF( XU(IU).EQ.XU(IU-1) .AND.
     &        XU(IU).EQ.XU(IU-2)      ) THEN
           LDONE = .FALSE.
           IUREM = IU
          ENDIF
        ENDDO
        IU = NU
        IF(XU(IU).EQ.XU(IU-1)) THEN
         LDONE = .FALSE.
         IUREM = IU
        ENDIF
C
        IF(LDONE) THEN
         GO TO 30
        ELSE
         DO IU = IUREM, NU-1
           XU(IU) = XU(IU+1)
           YU(IU) = YU(IU+1)
         ENDDO
         NU = NU - 1
        ENDIF
      ENDDO
C
C---- pick up here when no more points to be removed
 30   CONTINUE
      IF(NU.LT.2) THEN
       WRITE(*,*)
       WRITE(*,*) 'Need at least 2 points'
       GO TO 90
      ENDIF
C
C
C---- find which X,Y input point is closest to first-specified point
      ISMOD = 1
      IMOD1 = IFRST(ISMOD)
      XUI = (X(IMOD1)-XOFF)*XSF
      YUI = (Y(IMOD1)-YOFF)*YSF
      DSQMIN = (XUI-XUSP1)**2 + (YUI-YUSP1)**2
      DO IS = 1, NSIDE
        DO I = IFRST(IS), ILAST(IS)
C-------- convert input arrays to plot coordinates
          XUI = (X(I)-XOFF)*XSF
          YUI = (Y(I)-YOFF)*YSF
          DSQ = (XUI-XUSP1)**2 + (YUI-YUSP1)**2
C
          IF(DSQ .LT. DSQMIN) THEN
C---------- this point is the closest so far... note its indices
            DSQMIN = DSQ
            ISMOD = IS
            IMOD1 = I
          ENDIF
        ENDDO
      ENDDO
C
C---- set side and function to be modified
      IS = ISMOD
C
C---- find which X,Y input point is closest to last-specified point,
C-    but check only element IS
      IMOD2 = IFRST(IS)
      XUI = (X(IMOD2)-XOFF)*XSF
      YUI = (Y(IMOD2)-YOFF)*YSF
      DSQMIN = (XUI-XUSP2)**2 + (YUI-YUSP2)**2
      DO I = IFRST(IS), ILAST(IS)
C------ convert input arrays to plot coordinates
        XUI = (X(I)-XOFF)*XSF
        YUI = (Y(I)-YOFF)*YSF
        DSQ = (XUI-XUSP2)**2 + (YUI-YUSP2)**2
C
        IF(DSQ .LT. DSQMIN) THEN
C-------- this point is the closest so far... note its indices
          DSQMIN = DSQ
          IMOD2 = I
        ENDIF
      ENDDO
C
      IF    (IMOD1.EQ.IMOD2) THEN
       WRITE(*,*)
       WRITE(*,*) 'Graft endpoints must be distinct'
       GO TO 90
      ELSEIF(IMOD1.GT.IMOD2) THEN
C----- reverse the input-point ordering to get increasing S values
       DO IU = 1, NU/2
         XTMP = XU(IU)
         YTMP = YU(IU)
         XU(IU) = XU(NU-IU+1)
         YU(IU) = YU(NU-IU+1)
         XU(NU-IU+1) = XTMP
         YU(NU-IU+1) = YTMP
       ENDDO
       ITMP = IMOD1
       IMOD1 = IMOD2
       IMOD2 = ITMP
      ENDIF
C
C---- reset X,Y and dX/dS,dY/dS at first and last points of modified interval
      IU = 1
      IF(LBLEND .OR. IMOD1.NE.IFRST(IS)) THEN
C----- reset 1st input point to match contour, except if non-blended endpoint
       XU(IU) = X(IMOD1)
       YU(IU) = Y(IMOD1)
      ENDIF
      IF(LBLEND .AND. IMOD1.NE.IFRST(IS)) THEN
C----- match derivatives to current contour, except at the endpoints
       XUD1 = XD(IMOD1)
       YUD1 = YD(IMOD1)
      ELSE
C----- do not constrain 1st derivatives (set zero 3rd derivative instead)
       XUD1 = -999.0
       YUD1 = -999.0
      ENDIF
C
      IU = NU
      IF(LBLEND .OR. IMOD2.NE.ILAST(IS)) THEN
C----- reset 1st input point to match contour, except if non-blended endpoint
       XU(IU) = X(IMOD2)
       YU(IU) = Y(IMOD2)
      ENDIF
      IF(LBLEND .AND. IMOD2.NE.ILAST(IS)) THEN
C----- match derivatives to current contour
       XUD2 = XD(IMOD2)
       YUD2 = YD(IMOD2)
      ELSE
C----- do not constrain 1st derivatives (set zero 3rd derivative instead)
       XUD2 = -999.0
       YUD2 = -999.0
      ENDIF
C
C---- set spline parameter
      CALL SCALC(XU,YU,SU,NU)
C
C---- shift and rescale spline parameter SU to match current S
      SU1 = SU(1)
      SU2 = SU(NU)
      DO IU = 1, NU
        SFRAC = (SU(IU)-SU1)/(SU2-SU1)
        SU(IU) = S(IMOD1)*(1.0-SFRAC) + S(IMOD2)*SFRAC
      ENDDO
C
C---- spline input function values
      CALL SEGSPLD(XU,XUD,SU,NU,XUD1,XUD2)
      CALL SEGSPLD(YU,YUD,SU,NU,YUD1,YUD2)
C
C
C---- go over all points on modified segment
      DO I = IMOD1, IMOD2
        SI = S(I)
        X(I) = SEVAL(SI,XU,XUD,SU,NU)
        Y(I) = SEVAL(SI,YU,YUD,SU,NU)
      ENDDO
C
      IF(LMODPL) THEN
C----- plot modified function over modified interval
       CALL NEWCOLORNAME('MAGENTA')
       IPEN = 3
       DO I = IMOD1, IMOD2
         XP = (X(I)-XOFF)*XSF
         YP = (Y(I)-YOFF)*YSF
         CALL PLOT(XP,YP,IPEN)
         IPEN = 2
       ENDDO
       CALL PLFLUSH
      ENDIF
C
C---- return normally
      CALL NEWCOLOR(ICOL0)
      RETURN
C
C-------------------------------------------------
 90   CONTINUE
      WRITE(*,*) 'No changes made'
      IMOD1 = IFRST(1)
      IMOD2 = IFRST(1) - 1
      ISMOD = 1
      CALL NEWCOLOR(ICOL0)
      RETURN
C
      END ! MODIXY



      SUBROUTINE KEYOFF(XCRS,YCRS,CHKEY, 
     &                  XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
      CHARACTER*1 CHKEY
      LOGICAL LPLNEW
C
      IKEY = ICHAR(CHKEY)
C
      LPLNEW = .FALSE.
C
      IF    (IKEY.EQ.81 .OR. IKEY.EQ.180) THEN
C----- pan left arrow
       XOFF = XOFF - 0.02/XSF
       LPLNEW = .TRUE.
C
      ELSEIF(IKEY.EQ.83 .OR. IKEY.EQ.182) THEN
C----- pan right arrow
       XOFF = XOFF + 0.02/XSF
       LPLNEW = .TRUE.
 
      ELSEIF(IKEY.EQ.82 .OR. IKEY.EQ.184) THEN
C----- pan up arrow
       YOFF = YOFF + 0.02/YSF
       LPLNEW = .TRUE.
 
      ELSEIF(IKEY.EQ.84 .OR. IKEY.EQ.178) THEN
C----- pan down arrow
       YOFF = YOFF - 0.02/YSF
       LPLNEW = .TRUE.
 
      ELSEIF(IKEY.EQ.85 .OR. IKEY.EQ.185) THEN
C----- zoom in (Page Up)
       XCEN = 0.5*XWS/XSF + XOFF
       YCEN = 0.5*YWS/YSF + YOFF
       XSF = 1.05*XSF
       YSF = 1.05*YSF
       XOFF = XCEN - 0.5*XWS/XSF
       YOFF = YCEN - 0.5*YWS/YSF
       LPLNEW = .TRUE.
 
      ELSEIF(IKEY.EQ.86 .OR. IKEY.EQ.179) THEN
C----- zoom out (Page Down)
       XCEN = 0.5*XWS/XSF + XOFF
       YCEN = 0.5*YWS/YSF + YOFF
       XSF = XSF/1.05
       YSF = YSF/1.05
       XOFF = XCEN - 0.5*XWS/XSF
       YOFF = YCEN - 0.5*YWS/YSF
       LPLNEW = .TRUE.
C
      ELSEIF(INDEX('Ii',CHKEY).NE.0)  THEN
C----- zoom in, keeping cursor point fixed
       XCU = XCRS/XSF + XOFF
       YCU = YCRS/YSF + YOFF
       XSF = XSF*1.075
       YSF = YSF*1.075
       XOFF = XCU - XCRS/XSF
       YOFF = YCU - YCRS/YSF
       LPLNEW = .TRUE.
 
      ELSEIF(INDEX('Oo',CHKEY).NE.0)  THEN
C----- zoom out, keeping cursor point fixed
       XCU = XCRS/XSF + XOFF
       YCU = YCRS/YSF + YOFF
       XSF = XSF/1.075
       YSF = YSF/1.075
       XOFF = XCU - XCRS/XSF
       YOFF = YCU - YCRS/YSF
       LPLNEW = .TRUE.
 
      ELSEIF(INDEX('Pp',CHKEY).NE.0)  THEN
C----- pan towards cursor
       XCEN = 0.5*XWS
       YCEN = 0.5*YWS
C
       DX = (XCRS-XCEN)/SQRT(XWS*YWS)
       DY = (YCRS-YCEN)/SQRT(XWS*YWS)
C
       XOFF = XOFF + 0.05*DX/XSF
       YOFF = YOFF + 0.05*DY/YSF
       LPLNEW = .TRUE.
 
      ENDIF
C
      RETURN
      END ! KEYOFF

