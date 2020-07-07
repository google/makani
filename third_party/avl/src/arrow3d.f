
      SUBROUTINE ARWSET(R,SN,SLEN,HLEN,RHEAD,NHEAD,RLINES,NLINES)
      REAL R(3), SN(3), RLINES(3,2,*)
C------------------------------------------------------------------------
C     Creates a 3D wireframe arrow with a conical arrowhead.
C     The last RLINES dimension must be at least 2*NHEAD+1
C
C     Input:
C        R(.)   location of arrow tail
C        SN(.)  unit vector along arrow
C        SLEN   length of arrowshaft 
C        HLEN   length of arrowhead  (total length is SLEN+HLEN)
C        RHEAD  arrowhead radius
C        NHEAD  number of "pie slices" forming conical arrowhead
C
C     Output:
C        RLINES(.1s)  first  point of line segment
C        RLINES(.2s)  second point of line segment
C        NLINES       number of line segments  ( = 2*NHEAD+1 )
C
C        The last segment (s=NLINES) is the arrowshaft
C------------------------------------------------------------------------
      REAL A(3), B(3)
      DATA PI / 3.1415926 /
C
      KMIN = 1
      IF(ABS(SN(2)) .LT. ABS(SN(KMIN))) KMIN = 2
      IF(ABS(SN(3)) .LT. ABS(SN(KMIN))) KMIN = 3
C
      B(1) = 0.
      B(2) = 0.
      B(3) = 0.
      B(KMIN) = 1.0
C
      CALL CROSS(B,SN,A)
      AMAG = SQRT(A(1)**2 + A(2)**2 + A(3)**2)
      IF(AMAG .GT. 0.0) THEN
       A(1) = RHEAD*A(1)/AMAG
       A(2) = RHEAD*A(2)/AMAG
       A(3) = RHEAD*A(3)/AMAG
      ENDIF
C
      CALL CROSS(A,SN,B)
C
C---- set arrowhead perimeter lin segments
      IL = 1
      RLINES(1,1,IL) = R(1) + A(1)
      RLINES(2,1,IL) = R(2) + A(2)
      RLINES(3,1,IL) = R(3) + A(3)
C
      NT = NHEAD
      DT = 2.0*PI / FLOAT(NT)
      DO IT = 1, NT-1
        T = DT * FLOAT(IT)
        COST = COS(T)
        SINT = SIN(T)
C
        IL = IT
        RLINES(1,2,IL) = R(1) + A(1)*COST + B(1)*SINT
        RLINES(2,2,IL) = R(2) + A(2)*COST + B(2)*SINT
        RLINES(3,2,IL) = R(3) + A(3)*COST + B(3)*SINT
C
        RLINES(1,1,IL+1) = RLINES(1,2,IL)
        RLINES(2,1,IL+1) = RLINES(2,2,IL)
        RLINES(3,1,IL+1) = RLINES(3,2,IL)
      ENDDO
C
      IL = NT
      RLINES(1,2,IL) = RLINES(1,1,1)
      RLINES(2,2,IL) = RLINES(2,1,1)
      RLINES(3,2,IL) = RLINES(3,1,1)
      NLINES = NT
C
C---- set arrowhead radial segments
      DO IT = 1, NT
        IL = NT + IT
C
C------ from point on perimeter ...
        RLINES(1,1,IL) = RLINES(1,1,IT)
        RLINES(2,1,IL) = RLINES(2,1,IT)
        RLINES(3,1,IL) = RLINES(3,1,IT)
C
C------  ... to arrowhead point
        RLINES(1,2,IL) = R(1) + SN(1)*HLEN
        RLINES(2,2,IL) = R(2) + SN(2)*HLEN
        RLINES(3,2,IL) = R(3) + SN(3)*HLEN
      ENDDO
C
      NLINES = 2*NT
C
C---------------------------------------------------
C---- add arrowshaft
C
C---- move entire arrowhead by shaft distance
      DO IL = 1, NLINES
        DO K = 1, 3
          RLINES(K,1,IL) = RLINES(K,1,IL) + SLEN*SN(K)
          RLINES(K,2,IL) = RLINES(K,2,IL) + SLEN*SN(K)
        ENDDO
      ENDDO
C
C---- add shaft segment
      NLINES = NLINES + 1
      IL = NLINES
C
      RLINES(1,1,IL) = R(1)
      RLINES(2,1,IL) = R(2)
      RLINES(3,1,IL) = R(3)
C
      RLINES(1,2,IL) = R(1) + SLEN*SN(1)
      RLINES(2,2,IL) = R(2) + SLEN*SN(2)
      RLINES(3,2,IL) = R(3) + SLEN*SN(3)
C
      RETURN
      END ! ARWSET



      SUBROUTINE ARWPLT(XO,YO,AFAC,RLINES,SN,RHEAD,NHEAD)
C---------------------------------------------------------
C     Plots the 3D arrow created by ARWSET, presumably
C     after it was projected into screen x,y coordinates.
C     The z coordinates are used here for hidden-line 
C     logic to give the arrow a "solid" appearance.
C---------------------------------------------------------
      REAL RLINES(3,2,*), SN(3)
C
      REAL DLHEAD(3)
C
C---- end of arrowshaft
      NL = 2*NHEAD + 1
      ZSHAFT = RLINES(3,2,NL)
C
C---- arrowshaft length
      SSHAFT = SQRT(  (RLINES(1,2,NL)-RLINES(1,1,NL))**2
     &              + (RLINES(2,2,NL)-RLINES(2,1,NL))**2
     &              + (RLINES(3,2,NL)-RLINES(3,1,NL))**2 )
C
C---- normalize direction vector just in case it was deformed by perspective
      SNSQ = SN(1)**2 + SN(2)**2 + SN(3)**2
      IF(SNSQ .GT. 0.0) THEN
       SNI = 1.0/SQRT(SNSQ)
      ELSE
       SNI = 1.0
      ENDIF
C
C---- plot arrowhead perimiter
      DO IH = 1, NHEAD
        IL = IH
C------ midpoint of perimter segment
        ZMID = (RLINES(3,2,IL) + RLINES(3,1,IL))*0.5
C
C------ plot segment only if arrow points down (z component into screen),
C        or arrowshaft end's z location is below segment's z midpoint
        IF(SN(3) .LT. 0.0 .OR. ZSHAFT .LT. ZMID) THEN
         X1 = XO + AFAC*RLINES(1,1,IL)
         Y1 = YO + AFAC*RLINES(2,1,IL)
         X2 = XO + AFAC*RLINES(1,2,IL)
         Y2 = YO + AFAC*RLINES(2,2,IL)
         CALL PLOT(X1,Y1,3)
         CALL PLOT(X2,Y2,2)
        ENDIF
      ENDDO
C
C---- plot arrowhead radial segments
      DO IH = 1, NHEAD
        IL = NHEAD + IH
C
C------ set unit vector along radial segment
        DLHEAD(1) = RLINES(1,2,IL) - RLINES(1,1,IL)
        DLHEAD(2) = RLINES(2,2,IL) - RLINES(2,1,IL)
        DLHEAD(3) = RLINES(3,2,IL) - RLINES(3,1,IL)
        SLHEAD = SQRT(DLHEAD(1)**2 + DLHEAD(2)**2 + DLHEAD(3)**2)
        IF(SLHEAD .GT. 0.0) THEN
         DLHEAD(1) = DLHEAD(1)/SLHEAD
         DLHEAD(2) = DLHEAD(2)/SLHEAD
         DLHEAD(3) = DLHEAD(3)/SLHEAD
        ENDIF
C
C------ plot segment only if it points more into screen than arrowshaft
        IF(DLHEAD(3) .LT. SN(3)*SNI) THEN
         X1 = XO + AFAC*RLINES(1,1,IL)
         Y1 = YO + AFAC*RLINES(2,1,IL)
         X2 = XO + AFAC*RLINES(1,2,IL)
         Y2 = YO + AFAC*RLINES(2,2,IL)
         CALL PLOT(X1,Y1,3)
         CALL PLOT(X2,Y2,2)
        ENDIF
      ENDDO
C
C---- don't bother plotting zero-length arrowshaft
      IF(SSHAFT .EQ. 0.0) RETURN
C
C---- find visible fraction of arrowshaft
      SN12 = SQRT(SN(1)**2 + SN(2)**2)
      IF(SN12 .EQ. 0.0) THEN
       TANT = 0.0
      ELSE
       TANT = SN(3) / SN12
      ENDIF
      SSEEN = MAX( SSHAFT - RHEAD*MAX( TANT , 0.0 ) , 0.0 )
C
      IF(SSEEN .EQ. 0.0) RETURN
C
C---- plot visible fraction of arrowshaft
      SFRAC = SSEEN/SSHAFT
C
      IL = NL
      X1 = XO + AFAC* RLINES(1,1,IL)
      Y1 = YO + AFAC* RLINES(2,1,IL)
      X2 = X1 + AFAC*(RLINES(1,2,IL)-RLINES(1,1,IL))*SFRAC
      Y2 = Y1 + AFAC*(RLINES(2,2,IL)-RLINES(2,1,IL))*SFRAC
      CALL PLOT(X1,Y1,3)
      CALL PLOT(X2,Y2,2)
C
      RETURN
      END ! ARWPLT

