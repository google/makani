C
      SUBROUTINE CONLAB(IX,JX,II,JJ,X,Y,F,FCON,XWT,YWT,
     &           CH,NDIG,ISIDE)
      DIMENSION X(IX,JX), Y(IX,JX)
      DIMENSION F(IX,JX)
C---------------------------------------------------------------------
C     Puts numerical labels on the contour with value FCON at the
C     edge of the domain specified by ISIDE.  The number of digits
C     in the label(s) after the decimal point is given by NDIG.
C
C     Input:
C        IX JX     dimensions of arrays   X, Y, F
C        II JJ     array limits of arrays X, Y, F
C        X(i,j)    coordinates of grid point (i,j)
C        Y(i,j)  
C        F(i,j)    function value at grid point (i,j)
C        FCON      value of F on the contour to be labeled
C        XWT YWT   plotting scale factors for X,Y:
C                       Xplot = X(i,j)*XWT
C                       Yplot = Y(i,j)*YWT
C        CH        absolute character height (no scaling is done)
C        NDIG      number of digits after decimal point in labels
C        ISIDE     domain side on which labels are to appear:
C
C           .     3     .
C
C           4           2
C
C           .     1     .
C
C
C     Output:      direct plotting calls using Versatec routines
C
C---------------------------------------------------------------------
      LOGICAL LABEL 
C
      DATA PI, RTOD / 3.141592654, 57.2957795 /
C
C---- total number of digits + decimal point
      RDIG = 3.25 + FLOAT(NDIG)
C
      IF(ISIDE.EQ.1) THEN
       JO = 1
       JP = 2
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.2) THEN
       IO = II-1
       IP = II
       KLO = 1
       KHI = JJ-1
      ELSE IF(ISIDE.EQ.3) THEN
       JO = JJ-1
       JP = JJ
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.4) THEN
       IO = 1
       IP = 2
       KLO = 1
       KHI = JJ-1
      ENDIF
C
C---- check domain edge specified by ISIDE if the contour touches it
      DO 10 K=KLO, KHI
C
        IF(ISIDE.EQ.1) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.2) THEN
         JO = K
         JP = K+1
        ELSE IF(ISIDE.EQ.3) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.4) THEN
         JO = K
         JP = K+1
        ENDIF
C
C------ flag indicating if contour crosses current cell
        LABEL = .FALSE.
C
C           op    3    pp
C
C           4           2
C
C           oo    1    po
C
        XOO = X(IO,JO)
        XOP = X(IO,JP)
        XPO = X(IP,JO)
        XPP = X(IP,JP)
C
        YOO = Y(IO,JO)
        YOP = Y(IO,JP)
        YPO = Y(IP,JO)
        YPP = Y(IP,JP)
C
        FOO = F(IO,JO)
        FOP = F(IO,JP)
        FPO = F(IP,JO)
        FPP = F(IP,JP)
C
C------ bottom edge (side 1)
        IF(FCON.GE.FOO .AND. FCON.LT.FPO .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FPO     ) THEN
         IF(ISIDE.EQ.1) THEN
          XCON2 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON2 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON1 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
         ENDIF 
        ENDIF
C
C------ left edge (side 4)
        IF(FCON.GE.FOO .AND. FCON.LT.FOP .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FOP     ) THEN
         IF(ISIDE.EQ.4) THEN
          XCON2 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON2 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON1 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
         ENDIF 
        ENDIF
C
C------ right edge (side 2)
        IF(FCON.GE.FPO .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FPO .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.2) THEN
          XCON2 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON2 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON1 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
         ENDIF 
        ENDIF
C
C------ top edge (side 3)
        IF(FCON.GE.FOP .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FOP .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.3) THEN
          XCON2 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON2 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON1 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
         ENDIF 
        ENDIF
C
        IF(LABEL) THEN
C
C------- a contour reaching the domain edge has been found - set coordinates
C        of contour on the cell edges
         X1 = XWT*XCON1
         X2 = XWT*XCON2
         Y1 = YWT*YCON1
         Y2 = YWT*YCON2
C
         DX = X2 - X1
         DY = Y2 - Y1
C
C------- contour angle
         ACON = ATAN2( DY , DX )
         SA = SIN(ACON)
         CA = COS(ACON)
C
C------- if contour points to the right ...
         IF(ABS(ACON) .LT. 0.5*PI) THEN
C
C-------- set angle and lower left coordinates of number
          ANUM = RTOD*ACON
          XN = X2 + CH*CA + 0.5*CH*SA
          YN = Y2 + CH*SA - 0.5*CH*CA
C
C------- if contour points to the left ...
         ELSE
C
C-------- add +/- 180 degrees to number angle to make it read right to left
          ANUM = RTOD*ACON - SIGN(180.0,ACON)
C
C-------- set lower left coordinates of number
          XN = X2 + CH*RDIG*CA - 0.5*CH*SA
          YN = Y2 + CH*RDIG*SA + 0.5*CH*CA
C
         ENDIF
C
C------- draw number
         CALL NUMBER(XN,YN,CH,FCON,ANUM,NDIG)
C
        ENDIF
C
   10 CONTINUE
C
      RETURN
      END ! CONLAB



      SUBROUTINE CONLABC(IX,JX,II,JJ,X,Y,F,FCON,CLAB,XWT,YWT,
     &           CH,NDIG,ISIDE)
      DIMENSION X(IX,JX), Y(IX,JX)
      DIMENSION F(IX,JX)
      CHARACTER*(*) CLAB
C---------------------------------------------------------------------
C     Puts numerical labels on the contour with value FCON at the
C     edge of the domain specified by ISIDE.  The number of digits
C     in the label(s) after the decimal point is given by NDIG.
C
C     Input:
C        IX JX     dimensions of arrays   X, Y, F
C        II JJ     array limits of arrays X, Y, F
C        X(i,j)    coordinates of grid point (i,j)
C        Y(i,j)  
C        F(i,j)    function value at grid point (i,j)
C        FCON      value of F on the contour to be labeled
C        CLAB      label string
C        XWT YWT   plotting scale factors for X,Y:
C                       Xplot = X(i,j)*XWT
C                       Yplot = Y(i,j)*YWT
C        CH        absolute character height (no scaling is done)
C        NDIG      number of digits after decimal point in labels
C        ISIDE     domain side on which labels are to appear:
C
C           .     3     .
C
C           4           2
C
C           .     1     .
C
C
C     Output:      direct plotting calls using Versatec routines
C
C---------------------------------------------------------------------
      LOGICAL LABEL 
C
      DATA PI, RTOD / 3.141592654, 57.2957795 /
C
C---- total number of digits + decimal point
      RDIG = 3.25 + FLOAT(NDIG)
C
      IF(ISIDE.EQ.1) THEN
       JO = 1
       JP = 2
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.2) THEN
       IO = II-1
       IP = II
       KLO = 1
       KHI = JJ-1
      ELSE IF(ISIDE.EQ.3) THEN
       JO = JJ-1
       JP = JJ
       KLO = 1
       KHI = II-1
      ELSE IF(ISIDE.EQ.4) THEN
       IO = 1
       IP = 2
       KLO = 1
       KHI = JJ-1
      ENDIF
C
C---- check domain edge specified by ISIDE if the contour touches it
      DO 10 K=KLO, KHI
C
        IF(ISIDE.EQ.1) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.2) THEN
         JO = K
         JP = K+1
        ELSE IF(ISIDE.EQ.3) THEN
         IO = K
         IP = K+1
        ELSE IF(ISIDE.EQ.4) THEN
         JO = K
         JP = K+1
        ENDIF
C
C------ flag indicating if contour crosses current cell
        LABEL = .FALSE.
C
C           op    3    pp
C
C           4           2
C
C           oo    1    po
C
        XOO = X(IO,JO)
        XOP = X(IO,JP)
        XPO = X(IP,JO)
        XPP = X(IP,JP)
C
        YOO = Y(IO,JO)
        YOP = Y(IO,JP)
        YPO = Y(IP,JO)
        YPP = Y(IP,JP)
C
        FOO = F(IO,JO)
        FOP = F(IO,JP)
        FPO = F(IP,JO)
        FPP = F(IP,JP)
C
C------ bottom edge (side 1)
        IF(FCON.GE.FOO .AND. FCON.LT.FPO .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FPO     ) THEN
         IF(ISIDE.EQ.1) THEN
          XCON2 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON2 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XPO-XOO)/(FPO-FOO)
          YCON1 = YOO + (FCON-FOO)*(YPO-YOO)/(FPO-FOO)
         ENDIF 
        ENDIF
C
C------ left edge (side 4)
        IF(FCON.GE.FOO .AND. FCON.LT.FOP .OR.
     &     FCON.LT.FOO .AND. FCON.GE.FOP     ) THEN
         IF(ISIDE.EQ.4) THEN
          XCON2 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON2 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOO + (FCON-FOO)*(XOP-XOO)/(FOP-FOO)
          YCON1 = YOO + (FCON-FOO)*(YOP-YOO)/(FOP-FOO)
         ENDIF 
        ENDIF
C
C------ right edge (side 2)
        IF(FCON.GE.FPO .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FPO .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.2) THEN
          XCON2 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON2 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
          LABEL = .TRUE.
         ELSE
          XCON1 = XPO + (FCON-FPO)*(XPP-XPO)/(FPP-FPO)
          YCON1 = YPO + (FCON-FPO)*(YPP-YPO)/(FPP-FPO)
         ENDIF 
        ENDIF
C
C------ top edge (side 3)
        IF(FCON.GE.FOP .AND. FCON.LT.FPP .OR.
     &     FCON.LT.FOP .AND. FCON.GE.FPP     ) THEN
         IF(ISIDE.EQ.3) THEN
          XCON2 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON2 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
          LABEL = .TRUE.
         ELSE
          XCON1 = XOP + (FCON-FOP)*(XPP-XOP)/(FPP-FOP)
          YCON1 = YOP + (FCON-FOP)*(YPP-YOP)/(FPP-FOP)
         ENDIF 
        ENDIF
C
        IF(LABEL) THEN
C
C------- a contour reaching the domain edge has been found - set coordinates
C        of contour on the cell edges
         X1 = XWT*XCON1
         X2 = XWT*XCON2
         Y1 = YWT*YCON1
         Y2 = YWT*YCON2
C
         DX = X2 - X1
         DY = Y2 - Y1
C
C------- contour angle
         ACON = ATAN2( DY , DX )
         SA = SIN(ACON)
         CA = COS(ACON)
C
C------- if contour points to the right ...
         IF(ABS(ACON) .LT. 0.5*PI) THEN
C
C-------- set angle and lower left coordinates of number
          ANUM = RTOD*ACON
          XN = X2 + CH*CA + 0.5*CH*SA
          YN = Y2 + CH*SA - 0.5*CH*CA
C
C------- if contour points to the left ...
         ELSE
C
C-------- add +/- 180 degrees to number angle to make it read right to left
          ANUM = RTOD*ACON - SIGN(180.0,ACON)
C
C-------- set lower left coordinates of number
          XN = X2 + CH*RDIG*CA - 0.5*CH*SA
          YN = Y2 + CH*RDIG*SA + 0.5*CH*CA
C
         ENDIF
C
C------- draw number
         CALL PLCHAR(XN,YN,CH,CLAB,ANUM,-1)
C
        ENDIF
C
   10 CONTINUE
C
      RETURN
      END ! CONLABC
