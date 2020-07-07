C***********************************************************************
C    Module:  dplot.f
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

      SUBROUTINE DPLOT(NPR1,XPR,YPR)
      INCLUDE 'XFOIL.INC'     
C-----------------------------------------------------------
C     Plots analytical profiles at specified points.
C     If NPR=0, then cursor-selected points are requested.
C-----------------------------------------------------------
      DIMENSION XPR(*), YPR(*)
C
      CHARACTER*1 KCHAR
      LOGICAL LCRS, TURB
      LOGICAL LGUI
C
      CALL GETCOLOR(ICOL0)
C
      LCRS = NPR1 .LE. 0
C
      IF(LCRS) THEN
       KDONE  = 1
       XDWIN = XPAGE - 2.0*XMARG
       YDWIN = YPAGE - 2.0*YMARG
       X1 = XMARG + 0.91*XDWIN
       X2 = XMARG + 0.99*XDWIN
       Y1 = YMARG + 0.01*YDWIN
       Y2 = YMARG + 0.05*YDWIN
       CALL NEWPEN(5)
       CALL GUIBOX(KDONE, X1,X2,Y1,Y2, 'GREEN'   , ' Done ')
C
       WRITE(*,*) ' '
       WRITE(*,*) 'Locate profiles with cursor, type "D" when done...'
       NPR = 12345
C
      ELSE
       NPR = NPR1
C
      ENDIF
C
C---- go over profiles ...
      DO 50 IPR=1, NPR
C
        IF(LCRS) THEN 
C------- get cursor plot coordinates
         CALL GETCURSORXY(XC,YC,KCHAR)
         IF(INDEX('Dd',KCHAR).NE.0 .OR. LGUI(KDONE,XC,YC)) THEN
          RETURN
         ENDIF
C
C------- transform to airfoil coordinates
         XC = XC/FACA - XOFA
         YC = YC/FACA - YOFA
C
        ELSE
         XC = XPR(IPR)
         YC = YPR(IPR)
C
        ENDIF
C
C------ find nearest airfoil surface point
        RSQMIN = 1.0E23
        ISMIN = 0
        IBLMIN = 0
        DOFF = 0.00001*(S(N)-S(1))
        DO IS = 1, 2
          DO IBL = 2, IBLTE(IS)
            I = IPAN(IBL,IS)
            XSURF = X(I) + DOFF*YP(I)
            YSURF = Y(I) - DOFF*XP(I)
            RSQ = (XC-XSURF)**2 + (YC-YSURF)**2
            IF(RSQ .LE. RSQMIN) THEN
             RSQMIN = RSQ
             ISMIN = IS
             IBLMIN = IBL
            ENDIF
          ENDDO
        ENDDO
C
        IS = ISMIN
        IBL = IBLMIN
C
        I = IPAN(IBL,IS)
        CRSP = (XC-X(I))*NY(I) - (YC-Y(I))*NX(I)
        IF(IS.EQ.2) CRSP = -CRSP
C
        IF(CRSP.GT.0.0) THEN
         IBLP = IBL+1
         IBLO = IBL
        ELSE
         IBLP = IBL
         IBLO = IBL-1
        ENDIF
        ISP = IS
        ISO = IS
C
        IF(IBLP.GT.IBLTE(IS)) THEN
         IBLP = IBLTE(IS)
         IBLO = IBLP-1
         IBL = IBLTE(IS)
        ELSEIF(IBLO.LT.2) THEN
         IBLO = 2
         IF(ISO.EQ.1) THEN
          ISO = 2
         ELSE
          ISO = 1
         ENDIF
        ENDIF
C
        IP = IPAN(IBLP,ISP)
        IO = IPAN(IBLO,ISO)
C
C------ set interpolation fraction at profile location
        DX = X(IP) - X(IO)
        DY = Y(IP) - Y(IO)
        VX = XC - X(IO)
        VY = YC - Y(IO)
        FRAC = (DX*VX + DY*VY)/(DX*DX+DY*DY)
        FRAC = MIN( MAX( FRAC , 0.0 ) , 1.0 )
C
C------ set averaged displacement vector at profile location
        CA = FRAC*NY(IP) + (1.0-FRAC)*NY(IO)
        SA = FRAC*NX(IP) + (1.0-FRAC)*NX(IO)
        CSMOD = SQRT(CA**2 + SA**2)
        CA = CA/CSMOD
        SA = SA/CSMOD
C
        X0 = FRAC*X(IP) + (1.0-FRAC)*X(IO)
        Y0 = FRAC*Y(IP) + (1.0-FRAC)*Y(IO)
C
        DS = FRAC*DSTR(IBLP,ISP) + (1.0-FRAC)*DSTR(IBLO,ISO)
        TH = FRAC*THET(IBLP,ISP) + (1.0-FRAC)*THET(IBLO,ISO)
        UE = FRAC*UEDG(IBLP,ISP) + (1.0-FRAC)*UEDG(IBLO,ISO)
C
        XI = FRAC*XSSI(IBLP,ISP) + (1.0-FRAC)*XSSI(IBLO,ISO)
        TURB = XI .GT. XSSITR(IS)
C
C------ 1 / (total enthalpy)
        HSTINV = GAMM1*(MINF/QINF)**2 / (1.0 + 0.5*GAMM1*MINF**2)
C
C------ Sutherland's const./To   (assumes stagnation conditions are at STP)
        HVRAT = 0.35
C
C------ fill Rtheta arrays
        UEC = UE * (1.0-TKLAM) / (1.0 - TKLAM*(UE/QINF)**2)
        HERAT = (1.0 - 0.5*HSTINV*UEC **2)
     &        / (1.0 - 0.5*HSTINV*QINF**2)
        RHOE = HERAT ** (1.0/GAMM1)
        AMUE = SQRT(HERAT**3) * (1.0+HVRAT)/(HERAT+HVRAT)
        RTHETA = REINF * RHOE*UE*TH/AMUE
C
        AMSQ = UEC*UEC*HSTINV / (GAMM1*(1.0 - 0.5*UEC*UEC*HSTINV))
        CALL HKIN( DS/TH, AMSQ, HK, DUMMY, DUMMY)
C
        WRITE(*,9100) X0,Y0, DS, RTHETA, HK
 9100   FORMAT(1X,'x y =', 2F8.4,'    Delta* =', G12.4,
     &         '    Rtheta =', F10.2,'    Hk =', F9.4)
C
        IF(IS.EQ.1) THEN
         UDIR = 1.0
        ELSE
         UDIR = -1.0
        ENDIF
C
        UEI = UE/QINF
        UN = 0.0
        CALL NEWCOLORNAME('green')
        UPRWTS = UPRWT*0.5*(S(N)-S(1))
        CALL PRPLOT(X0,Y0,TH,UEI,UN,HK,RTHETA,AMSQ,TURB,
     &              -XOFA,-YOFA,FACA,UPRWTS,SA,CA,UDIR)
   50 CONTINUE
C
      CALL NEWCOLOR(ICOL0)
      CALL PLFLUSH
C
      RETURN
      END ! DPLOT



      SUBROUTINE PRPLOT(X0,Y0,TH,UE,UN,HK,RET,MSQ,TURB,
     &                  XOFA,YOFA,FACA,UWT,SINA,COSA,UDIR)
C-----------------------------------------------------------------
C     Plots velocity profile taken from flow solution.
C
C   X0,Y0  coordinates of point through which profile axis passes
C   SA,CA  sin,cos of profile axis angle (cw from vertical)
C-----------------------------------------------------------------
      REAL MSQ
      LOGICAL TURB
C
      PARAMETER (KPRX=129)
      DIMENSION XX(KPRX), YY(KPRX), FFS(KPRX), SFS(KPRX)
c
      XMOD(XTMP) = FACA * (XTMP - XOFA)
      YMOD(YTMP) = FACA * (YTMP - YOFA)
C
      NN = KPRX
      UO = 1.0
      DK = HK*TH
      CT = 0.
C
      IF(TURB) THEN
C------ set Spalding + power-law turbulent profile
        CALL PRWALL(DK,TH,UO,RET,MSQ,CT, BB,
     &        DE, DE_DS, DE_TH, DE_UO, DE_RT, DE_MS,
     &        US, US_DS, US_TH, US_UO, US_RT, US_MS,
     &        HS, HS_DS, HS_TH, HS_UO, HS_RT, HS_MS,
     &        CF, CF_DS, CF_TH, CF_UO, CF_RT, CF_MS,
     &        CD, CD_DS, CD_TH, CD_UO, CD_RT, CD_MS,
     &            CD_CT  )
c
        CALL UWALL(TH,UO,DE,US,RET,CF,BB,   YY,XX,NN)
C
C------ limit profile height
        DECORR = 1.5 * (3.15 + 1.72/(HK-1.0) + HK) * TH
        DO 422 K=NN, 1, -1
          IF(YY(K) .LE. DECORR) GO TO 423
 422    CONTINUE
 423    NN = K
        DE = YY(K)
C
      ELSE
C------ set Falkner-Skan profile
        INORM = 3
        ISPEC = 2
        HSPEC = HK
        ETAE = 1.5*(3.15 + 1.72/(HK-1.0) + HK)
        GEO = 1.0
        CALL FS(INORM,ISPEC,BU,HSPEC,NN,ETAE,GEO,YY,FFS,XX,SFS,DEFS)
        DE = ETAE*TH
C
        DO 425 K=1, NN
          YY(K) = YY(K)*TH
 425    CONTINUE
C
      ENDIF
C
      YAX = 1.1*DE
C
      X1 = X0
      Y1 = Y0
      X2 = X0 + YAX*SINA
      Y2 = Y0 + YAX*COSA
C
C---- plot axis
      CALL NEWPEN(1)
      CALL PLOT(XMOD(X1),YMOD(Y1),3)
      CALL PLOT(XMOD(X2),YMOD(Y2),2)
C
      DO K=1, NN
        ULOC = UE + UN*(YY(K)-DK)
        XX(K) = XX(K)*UE * UWT * UDIR
CCC     YY(K) = YY(K)
      ENDDO
C
C---- rotate and position profile
      DO K=1, NN
        XBAR = XX(K)
        YBAR = YY(K)
        XROT = XBAR*COSA + YBAR*SINA + X0
        YROT = YBAR*COSA - XBAR*SINA + Y0
        XX(K) = XMOD(XROT)
        YY(K) = YMOD(YROT)
      ENDDO
C
      CALL NEWPEN(2)
      CALL XYLINE(NN,XX,YY,0.0,1.0,0.0,1.0,1)
C
      RETURN
      END ! PRPLOT

