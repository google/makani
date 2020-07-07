
 

      SUBROUTINE ADDP
C--------------------------------------------------
C     Adds cursor-selected point.
C--------------------------------------------------
      INCLUDE 'XFOIL.INC'
      REAL AINP(2)
      LOGICAL ERROR
C
      IF(NB.GE.IBX) THEN
       WRITE(*,*)
     &   'Buffer airfoil arrays will overflow.  No action taken.'
        RETURN
      ENDIF
C
      XWS = XWIND/SIZE
      YWS = YWIND/SIZE
C
  5   CONTINUE
C
C---- determine interval  IPNT-1...IPNT  which is to contain added point
      CALL POINTG(XB,XBP,YB,YBP,SB,NB, XWS,YWS, XOFF,YOFF,XSF,YSF,
     &            IPNT,AINP(1),AINP(2) )
      IF(IPNT.EQ.0) RETURN
C
      WRITE(*,*)
      WRITE(*,1020) ' New point', IPNT, AINP(1), AINP(2)
 1020 FORMAT(1X,A,I4,'      [ ',2F10.6,' ] :  ', $)
C
      CALL READR(2,AINP,ERROR)
      IF(ERROR) THEN
       WRITE(*,*) '* READ error.  No changes made.'
       RETURN
      ENDIF
C
C---- make room for new point
      DO I=NB, IPNT, -1
        XB(I+1) = XB(I)
        YB(I+1) = YB(I)
      ENDDO
      NB = NB+1
C
C---- set new point
      XB(IPNT) = AINP(1)
      YB(IPNT) = AINP(2)
C
      LGSAME = .FALSE.
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
      CALL NEWPEN(2)
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      LGEOPL = .FALSE.
C
      GO TO 5
ccc      RETURN
      END ! ADDP



      SUBROUTINE MOVP(NEWPLOTG)
C--------------------------------------------------
C     Moves cursor-selected point.
C--------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL LGUI
      CHARACTER*1 KCHAR
      REAL AINP(2)
      LOGICAL ERROR, LPLNEW
      EXTERNAL NEWPLOTG
      INCLUDE 'XDES.INC'
C
      SHT = 0.35*CH
C
      XWS = XWIND/SIZE
      YWS = YWIND/SIZE
C
 5    CONTINUE
      CALL POINTF(XB,XBP,YB,YBP,SB,NB, XWS,YWS, XOFF,YOFF,XSF,YSF,
     &            IPNT,XC,YC )
      IF(IPNT.EQ.0) RETURN
C
      CALL PLSYMB(XMOD(XB(IPNT)),YMOD(YB(IPNT)),SHT,1,0.0,0)
      CALL PLFLUSH
C
 1000 FORMAT(A)
 1010 FORMAT(1X,A,I4,'"o":  x,y =',2F10.6,A)
 1020 FORMAT(1X,A,I4,'"+" ?    [ ',2F10.6,' ] :  ', $)
C
      WRITE(*,*)
      WRITE(*,1010) 'Move point',  IPNT, XB(IPNT), YB(IPNT),
     &              '   to cursor click ...'
C
 10   CONTINUE
      CALL NEWPEN(5)
      KDONE = 1
      CALL PGUI(KDONE,'green','Done')
      CALL PLFLUSH
      CALL NEWPEN(1)
      CALL GETCURSORXY(XCRS,YCRS,KCHAR)
C
C---- check if zoom,pan action was requested
      CALL KEYOFF(XCRS,YCRS,KCHAR,
     &            XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
      IF(LPLNEW) THEN
C----- scales,offsets have changed... replot
       CALL NEWPLOTG
       CALL PLSYMB(XMOD(XB(IPNT)),YMOD(YB(IPNT)),SHT,1,0.0,0)
       CALL PLFLUSH
       GO TO 10
      ENDIF
C
C---- Done button pushed or "D" typed?
      IF(LGUI(KDONE,XCRS,YCRS) .OR. INDEX('Dd',KCHAR).NE.0) RETURN
C
C
C---- OK, new point was selected... first confirm with "+" symbol
      CALL PLSYMB(XCRS,YCRS,1.5*SHT,3,0.0,0)
      CALL PLFLUSH
C
C---- go from screen to internal coordinates X,Y
      AINP(1) = XCRS/XSF + XOFF
      AINP(2) = YCRS/YSF + YOFF
C
      WRITE(*,1020) 'New  point', IPNT, AINP(1), AINP(2)
      CALL READR(2,AINP,ERROR)
      IF(ERROR) THEN
       WRITE(*,*) '* READ error.  No changes made.'
       RETURN
      ENDIF
C
      XB(IPNT) = AINP(1)
      YB(IPNT) = AINP(2)
C
      LGSAME = .FALSE.
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
      CALL NEWPEN(2)
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      LGEOPL = .FALSE.
C
      GO TO 5
ccc      RETURN
      END ! MOVP



      SUBROUTINE DELP
C--------------------------------------------------
C     Deletes cursor-selected point.
C--------------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      XWS = XWIND/SIZE
      YWS = YWIND/SIZE
C
 5    CONTINUE
      CALL POINTF(XB,XBP,YB,YBP,SB,NB, XWS,YWS, XOFF,YOFF,XSF,YSF,
     &            IPNT,XC,YC )
      IF(IPNT.EQ.0) RETURN
C
C---- remove closest point
      DO I=IPNT, NB-1
        XB(I) = XB(I+1)
        YB(I) = YB(I+1)
      ENDDO
      NB = NB-1
C
      LGSAME = .FALSE.
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
      CALL NEWPEN(2)
      CALL PLTAIR(XB,XBP,YB,YBP,SB,NB, XOFF,XSF,YOFF,YSF,'magenta')
      LGEOPL = .FALSE.
      LGSAME = .FALSE.
C
      WRITE(*,1010) IPNT, XC, YC
 1010 FORMAT(/' Deleted point',I4,' :   x =',F10.6,'    y =',F10.6)
C
      GO TO 5
ccc      RETURN
      END ! DELP



      SUBROUTINE DIST
C--------------------------------------------------
C     Displays distance between two cursor points.
C--------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*1 KCHAR
C
      WRITE(*,*)
      WRITE(*,*) 'Click mouse or hit a key on each point'
      WRITE(*,*)
      CALL GETCURSORXY(XX1,YY1,KCHAR)
      CALL PLOT(XX1,YY1,3)
      CALL PLOT(XX1,YY1,2)
      CALL PLFLUSH
      XX1 = XX1/XSF + XOFF
      YY1 = YY1/YSF + YOFF
      WRITE(*,1010) XX1,YY1
C
      CALL GETCURSORXY(XX2,YY2,KCHAR)
      CALL PLOT(XX2,YY2,3)
      CALL PLOT(XX2,YY2,2)
      CALL PLFLUSH
      XX2 = XX2/XSF + XOFF
      YY2 = YY2/YSF + YOFF
      WRITE(*,1020) XX2,YY2
C
      DX = XX2 - XX1
      DY = YY2 - YY1
      DS = SQRT(DX*DX + DY*DY)
      WRITE(*,1050) DX, DY, DS
C
 1010 FORMAT(' x1 =', F10.6, '    y1 =', F10.6)
 1020 FORMAT(' x2 =', F10.6, '    y2 =', F10.6)
 1050 FORMAT(' dx =', F10.6, '    dy =', F10.6,'    ds =', F10.6)
C
      RETURN
      END ! DIST






      SUBROUTINE POINTF(X,XP,Y,YP,S,N, XWS,YWS, XOFF,YOFF,XSF,YSF, 
     &                  IC,XX,YY )
      DIMENSION X(N),XP(N),Y(N),YP(N),S(N)
      LOGICAL LGUI
C
      CHARACTER*1 KCHAR
      LOGICAL LPLNEW
C--------------------------------------------------------
C     Finds the node IC nearest to cursor location XX,YY.
C--------------------------------------------------------
CCC      XMOD(XTMP) = XSF * (XTMP - XOFF)
CCC      YMOD(YTMP) = YSF * (YTMP - YOFF)
C
      WRITE(*,*)
      WRITE(*,*) 'Specify point with cursor...'
      WRITE(*,*) 'Or.. Type I,O,P to In,Out,Pan with cursor...'
C
 10   CONTINUE
      CALL NEWPEN(5)
      KDONE = 1
      CALL PGUI(KDONE,'green','Done')
      CALL PLFLUSH
      CALL NEWPEN(1)
C
C---- read geometry point coordinates
      CALL GETCURSORXY(XCRS,YCRS,KCHAR)
C
C---- do possible pan,zoom operations based on KCHAR
      CALL KEYOFF(XCRS,YCRS,KCHAR, XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
      IF(LPLNEW) THEN
C----- scales,offsets have changed... replot
       CALL GOFSET
       CALL PLTINI
       CALL PLOTG
       GO TO 10
      ENDIF
C
      IF(LGUI(KDONE,XCRS,YCRS) .OR. INDEX('Dd',KCHAR).NE.0) THEN
C----- abort: return with point selected
       IC = 0
       RETURN
      ENDIF
C
C---- go from screen to internal coordinates X,Y
      XX = XCRS/XSF + XOFF
      YY = YCRS/YSF + YOFF
C
C---- find closest airfoil node
      IC = 1
      DMIN = 1.0E9
      DO 7 I=1, N
        DIST = (X(I) - XX)**2 + (Y(I) - YY)**2
        IF(DIST .LT. DMIN) THEN
          DMIN = DIST
          IC = I
        ENDIF
    7 CONTINUE
C
      RETURN
      END ! POINTF



      SUBROUTINE POINTG(X,XP,Y,YP,S,N, XWS,YWS, XOFF,YOFF,XSF,YSF,
     &                  IC,XX,YY )
      DIMENSION X(N),XP(N),Y(N),YP(N),S(N)
      LOGICAL LGUI
C
      CHARACTER*1 KCHAR
      LOGICAL LPLNEW
C--------------------------------------------------------
C     Finds the interval IC-1..IC with spline nearest
C     to cursor location XX,YY.
C--------------------------------------------------------
CCC      XMOD(XTMP) = XSF * (XTMP - XOFF)
CCC      YMOD(YTMP) = YSF * (YTMP - YOFF)
C
C---- number of spline sub-interval points searched
      DATA KK / 10 /
C
      WRITE(*,*)
      WRITE(*,*) 'Specify point with cursor...'
      WRITE(*,*) 'Or.. Type I,O,P to In,Out,Pan with cursor...'
C
 10   CONTINUE
      KDONE = 1
      CALL NEWPEN(5)
      CALL PGUI(KDONE,'green','Done')
      CALL PLFLUSH
      CALL NEWPEN(1)
C
C---- read geometry point coordinates
      CALL GETCURSORXY(XCRS,YCRS,KCHAR)
C
C---- do possible pan,zoom operations based on KCHAR
      CALL KEYOFF(XCRS,YCRS,KCHAR, XWS,YWS, XOFF,YOFF,XSF,YSF, LPLNEW)
C
      IF(LPLNEW) THEN
C----- scales,offsets have changed... replot
       CALL GOFSET
       CALL PLTINI
       CALL PLOTG
       GO TO 10
      ENDIF
C
      IF(LGUI(KDONE,XCRS,YCRS) .OR. INDEX('Dd',KCHAR).NE.0) THEN
C----- abort: return with point selected
       IC = 0
       RETURN
      ENDIF
C
C---- go from screen to internal coordinates X,Y
      XX = XCRS/XSF + XOFF
      YY = YCRS/YSF + YOFF
C
C---- find closest spline node
      IC = 2
      KC = 0
      DMIN = (X(1) - XX)**2 + (Y(1) - YY)**2
      DO 6 I=2, N
        DS = S(I) - S(I-1)
C
C------ skip zero-width spline interval
        IF(DS .EQ. 0.0) GO TO 6
C
C------ search sub-interval points
        DO 62 K=1, KK
          ST = S(I-1) + DS*FLOAT(K)/FLOAT(KK)
          XT = SEVAL(ST,X,XP,S,N)
          YT = SEVAL(ST,Y,YP,S,N)
          DIST = (XT - XX)**2 + (YT - YY)**2
          IF(DIST .LT. DMIN) THEN
            DMIN = DIST
            IC = I
            KC = K
          ENDIF
 62     CONTINUE
 6    CONTINUE
C
      IF(KC.EQ.KK .AND. IC.LT.N) THEN
C------ spline node is the nearest point -- see on which side we are
        DOTP = (X(IC)-XX)*XP(IC) + (Y(IC)-YY)*YP(IC)
        IF(DOTP .LT. 0.0) IC = IC + 1
      ENDIF
C
      RETURN
      END ! POINTG
