C***********************************************************************
C    Module:  profil.f
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

      SUBROUTINE PRWALL(DSTAR,THETA,UO,RT,MS,CT, BB,
     &                  DO, DO_DS, DO_TH, DO_UO, DO_RT, DO_MS,
     &                  UI, UI_DS, UI_TH, UI_UO, UI_RT, UI_MS,
     &                  HS, HS_DS, HS_TH, HS_UO, HS_RT, HS_MS,
     &                  CF, CF_DS, CF_TH, CF_UO, CF_RT, CF_MS,
     &                  CD, CD_DS, CD_TH, CD_UO, CD_RT, CD_MS, CD_CT )
      IMPLICIT REAL (A-H,M,O-Z)
C================================================================
C     Returns wall slip velocity and thickness of wall BL profile
C
C     Input:
C        DSTAR  kinematic displacement thickness
C        THETA  kinematic momentum thickness
C        RT     momentum thickness based on ue and THETA
C        MS     Mach^2 based on ue
C
C        UO     uo/ue outer velocity;  assumed = 1  in this version
C
C     Output:
C        BB     outer profile exponent
C        DO     thickness of profile deck
C        UI     inner "slip" velocity
C        CF     wall skin friction
C================================================================
C
      PARAMETER (N=65)
      DIMENSION ETA(N), UIP(N), UIP_DP(N), G(N), G_BB(N)
C
C---- pi/2 ,  2/pi
      DATA HPI, TOPI / 1.570796327 , 0.6366197723 /
C
      DATA T, SQT / 0.28 , 0.5291502622 /
C
C---- TCON = ( atan(T^1/2) / T^1/2  -  1/(1+T)) / (2T)  +  0.5/(1.0 + T) 
C                - atan(1/T^1/2) / 2T^1/2
      DATA TCON / -0.3864027035 /
C       
C---- slip velocity coefficient
      DATA AK / 0.09 /
C
C---- log-law constants
      DATA VKAP, VB / 0.40 , 5.0 /
C
      HK = DSTAR/THETA
C
      UO = 1.0
      BB = 1.0
C
C---- initialize variables
      CALL CFT(HK,RT,MS,CF,CF_HK,CF_RT,CF_MS)
      SGN = SIGN( 1.0 , CF )
      UT = SGN * SQRT(0.5*ABS(CF))
C
      UI = MIN( UT/AK * HPI , 0.90 )
      DO = HK*THETA / (1.0 - 0.5*(UO+UI))
C
      EBK = EXP(-VB*VKAP)
C
      DO 1000 ITER=1, 40
C
        SGN = SIGN( 1.0 , UT )
C
C------ set d+ = DP(UT DO ; RT TH)
        DP    = SGN * UT*RT*DO/THETA
        DP_DO = SGN * UT*RT   /THETA
        DP_UT = SGN *    RT*DO/THETA
C
        DP_TH = -DP/THETA
        DP_RT = SGN * UT   *DO/THETA
        DP_MS = 0.0
C
C------ estimate inner profile edge velocity Ui+ using log-law
        UPE = LOG(DP)/VKAP + VB
C
C------ converge exact Ui+ using Spalding formula
        DO 10 ITUP=1, 5
          UK = UPE*VKAP
          ARG = UK - VB*VKAP
          EXU = EXP(ARG)
          REZ  = UPE +  EXU - EBK*(1.0 + UK + UK*UK/2.0 + UK*UK*UK/6.0)
     &         - DP
          DP_U = 1.0 + (EXU - EBK*(1.0 + UK + UK*UK/2.0))*VKAP
C
          IF(ABS(REZ/DP) .LT. 1.0E-5) GO TO 11
C
          DUPE = -REZ/DP_U
          UPE = UPE + DUPE
 10     CONTINUE
        WRITE(*,*) 'PRWALL: Ue+ convergence failed,  Res =', REZ/DP
 11     CONTINUE
C
        UPE_DP = 1.0/DP_U
C
C            2      2        3      3
C------ set d y+/du+   and  d y+/du+   at BL edge
        DP_UU  = (EXU - EBK*(1.0 + UK))*VKAP**2
        DP_UUU = (EXU - EBK           )*VKAP**3
C
C------ set  du+/dy+  at BL edge
        UPD    = 1.0/DP_U
        UPD_DP = (-1.0/DP_U**3) * DP_UU
C
C             2      2
C------ set  d u+/dy+   at BL edge
CCC     UPD_DP = (-1.0/DP_U**3) * DP_UU
        UPDD    = UPD_DP
        UPDD_DP = (-1.0/DP_U**4) * DP_UUU
     &          + ( 3.0/DP_U**5) * DP_UU**2
C
C------ set coefficients for Spalding profile correction polynomial
        DC2    = 0.5*DP*DP*UPDD    - DP*UPD
        DC2_DP =        DP*UPDD    -    UPD
     &         + 0.5*DP*DP*UPDD_DP - DP*UPD_DP
C
        DC3    = -(  DP*DP*UPDD    - DP*UPD   ) / 3.0
        DC3_DP = -(2.0 *DP*UPDD    -    UPD   ) / 3.0
     &           -(  DP*DP*UPDD_DP - DP*UPD_DP) / 3.0
C
C------ set outer profile amplitude DUO
        DUO    = UO - UT*(UPE    + DC2    + DC3   )
        DUO_DP =    - UT*(UPE_DP + DC2_DP + DC3_DP)
C
        DUO_UT =    -    (UPE    + DC2    + DC3   )
     &         + DUO_DP*DP_UT
        DUO_DO = DUO_DP*DP_DO
C
        DUO_TH = DUO_DP*DP_TH
        DUO_RT = DUO_DP*DP_RT
        DUO_MS = DUO_DP*DP_MS
c
c        write(*,*) 'dUo', duo, duo_dp, duo_ut, duo_do
c        read(*,*) ddo, dut
c        if(ddo.ne.0.0 .or. dut.ne.0.0) then
c          do = do+ddo
c          ut = ut+dut
c          write(*,*) 'new', duo + duo_do*ddo + duo_ut*dut
c          go to 666
c        endif
C
C------ set wake profile coefficients
        BB1    =  3.0*(BB    +2.0)*(BB+3.0)/(BB+7.0)
        BB1_BB =  3.0*(BB*2.0+5.0         )/(BB+7.0) - BB1/(BB+7.0)
        BB2    = -5.0*(BB    +1.0)*(BB+3.0)/(BB+7.0)
        BB2_BB = -5.0*(BB*2.0+4.0         )/(BB+7.0) - BB2/(BB+7.0)
        BB3    =  2.0*(BB    +1.0)*(BB+2.0)/(BB+7.0)
        BB3_BB =  2.0*(BB*2.0+3.0         )/(BB+7.0) - BB3/(BB+7.0)
C
C------ fill eta coordinate and inner profile arrays
CCC     EXUPE = EXP(UPE*VKAP - VB*VKAP)
        EXUPE = EXU
C
        DEXU = (EXUPE - EBK)/FLOAT(N-1)
C
        I = 1
        ETA(I) = 0.0
        UIP(I)    = 0.0
        UIP_DP(I) = 0.0
        G(I)    = 0.0
        G_BB(I) = 0.0
C
        DO 20 I=2, N
ccc       EXU = EBK + DEXU*FLOAT(I-1)
          EXU = EBK + (DEXU - 0.75*DEXU*FLOAT(N-I)/FLOAT(N-1))
     &                *FLOAT(I-1)
C
CCC       UK = UP*VKAP
          UK = LOG(EXU) + VB*VKAP
C
          UP = UK/VKAP
C
C-------- set "inverse" Spalding profile  y+(u+)  and derivatives
          YP    =  UP +  EXU - EBK*(1.0 + UK + UK*UK/2.0 + UK*UK*UK/6.0)
          YP_U  = 1.0 + (EXU - EBK*(1.0 + UK + UK*UK/2.0))*VKAP
          YP_UU =       (EXU - EBK*(1.0 + UK            ))*VKAP**2
C
          ET = YP/DP
C
C-------- set final inner profile (fudged Spalding)
          UIP(I)     = UP       +     DC2   *ET**2 +     DC3   *ET**3
          UIP_DP(I)  =                DC2_DP*ET**2 +     DC3_DP*ET**3
C
ccc          UIPD(I)    = 1.0/YP_U + 2.0*DC2   *ET    + 3.0*DC3   *ET**2
ccc          UIPD_DP(I) = (-1.0/YP_U**3)*YPUU
ccc     &                          + 2.0*DC2_DP*ET    + 3.0*DC3_DP*ET**2
C
C-------- set outer profile
          ETB = ET**BB
          ALE = LOG(ET)
C
ccc          G(I)    =  2.0*ETB -     ETB**2
ccc          G_BB(I) = (2.0*ETB - 2.0*ETB**2)*ALE
C
          G(I)    = (BB1   *ET + BB2   *ET**2 + BB3   *ET**3)*ETB
          G_BB(I) = (BB1_BB*ET + BB2_BB*ET**2 + BB3_BB*ET**3)*ETB
     &            + G(I)*ALE
C
          ETA(I) = ET
C
 20     CONTINUE
C
C
        DSN    = 0.0
        DSN_DO = 0.0
        DSN_UT = 0.0
        DSN_BB = 0.0
C
        DSN_TH = 0.0
        DSN_RT = 0.0
        DSN_MS = 0.0
C
C
        THN    = 0.0
        THN_DO = 0.0
        THN_UT = 0.0
        THN_BB = 0.0
C
        THN_TH = 0.0
        THN_RT = 0.0
        THN_MS = 0.0
C
c        TSN    = 0.0
c        TSN_DO = 0.0
c        TSN_UT = 0.0
c        TSN_BB = 0.0
c
c        TSN_TH = 0.0
c        TSN_RT = 0.0
c        TSN_MS = 0.0
C
C------ perform integration
        DO 100 I=1, N-1
          DETA = ETA(I+1) - ETA(I)
          GA    = 0.5*(G(I+1)    + G(I)   )
          GA_BB = 0.5*(G_BB(I+1) + G_BB(I))
C
          UIPA    = 0.5*(UIP(I+1)    + UIP(I)   )
          UIPA_DP = 0.5*(UIP_DP(I+1) + UIP_DP(I))
C
          U    = UT*UIPA    + DUO   *GA
          U_DP = UT*UIPA_DP
C
          U_DO =              DUO_DO*GA + U_DP*DP_DO
          U_UT =    UIPA    + DUO_UT*GA + U_DP*DP_UT
          U_BB =              DUO   *GA_BB
C
          U_TH =              DUO_TH*GA + U_DP*DP_TH
          U_RT =              DUO_RT*GA + U_DP*DP_RT
          U_MS =              DUO_MS*GA + U_DP*DP_MS
C
C
          DSN    = DSN    + (1.0 - U   )*DETA
          DSN_DO = DSN_DO -        U_DO *DETA
          DSN_UT = DSN_UT -        U_UT *DETA
          DSN_BB = DSN_BB -        U_BB *DETA
C
          DSN_TH = DSN_TH -        U_TH *DETA
          DSN_RT = DSN_RT -        U_RT *DETA
          DSN_MS = DSN_MS -        U_MS *DETA
C
C
          THN    = THN    + (U   -   U*U)     *DETA
          THN_DO = THN_DO + (1.0 - 2.0*U)*U_DO*DETA
          THN_UT = THN_UT + (1.0 - 2.0*U)*U_UT*DETA
          THN_BB = THN_BB + (1.0 - 2.0*U)*U_BB*DETA
C
          THN_TH = THN_TH + (1.0 - 2.0*U)*U_TH*DETA
          THN_RT = THN_RT + (1.0 - 2.0*U)*U_RT*DETA
          THN_MS = THN_MS + (1.0 - 2.0*U)*U_MS*DETA
C
c          TSN    = TSN    + (U   -   U*U*U)     *DETA
c          TSN_DO = TSN_DO + (1.0 - 3.0*U*U)*U_DO*DETA
c          TSN_UT = TSN_UT + (1.0 - 3.0*U*U)*U_UT*DETA
c          TSN_BB = TSN_BB + (1.0 - 3.0*U*U)*U_BB*DETA
C
c          TSN_TH = TSN_TH + (1.0 - 3.0*U*U)*U_TH*DETA
c          TSN_RT = TSN_RT + (1.0 - 3.0*U*U)*U_RT*DETA
c          TSN_MS = TSN_MS + (1.0 - 3.0*U*U)*U_MS*DETA
C
 100    CONTINUE
C
C------ set up 2x2 system for DO UT
        REZ1 = DO*DSN    - THETA*HK
        A11  = DO*DSN_DO + DSN
        A12  = DO*DSN_UT
cc        A12  = DO*DSN_BB
C
        REZ2 = DO*THN    - THETA
        A21  = DO*THN_DO + THN
        A22  = DO*THN_UT
cc        A22  = DO*THN_BB
C
        IF(ABS(REZ1/THETA) .LT. 2.0E-5 .AND.
     &     ABS(REZ2/THETA) .LT. 2.0E-5      ) GO TO 1010
c        IF(ABS(REZ1/THETA) .LT. 1.0E-3 .AND.
c     &     ABS(REZ2/THETA) .LT. 1.0E-3      ) GO TO 1010
C
        DET = A11*A22 - A12*A21
        B11 =  A22/DET
        B12 = -A12/DET
        B21 = -A21/DET
        B22 =  A11/DET
C
        DDO = -(B11*REZ1 + B12*REZ2)
        DUT = -(B21*REZ1 + B22*REZ2)
cc        DBB = -(B21*REZ1 + B22*REZ2)
C
        DMAX = MAX( ABS(DDO/DO) , ABS(DUT/0.05) )
cc        DMAX = MAX( ABS(DDO/DO) , ABS(DBB/BB  ) )
        RLX = 1.0
        IF(DMAX.GT.0.5) RLX = 0.5/DMAX
C
        DO = DO + RLX*DDO
        UT = UT + RLX*DUT
cc        BB = BB + RLX*DBB
c
c        write(*,4400) iter, do, ut, rez1, rez2, rlx
cc        write(*,*) iter, do, bb, rez1, rez2
 4400   format(1x,i5,f10.4,f11.6,2e12.4,f8.4)
C
 1000 CONTINUE
C
      WRITE(*,*) 'PRWALL: Convergence failed. Res =', REZ1, REZ2
C
 1010 CONTINUE
C
C
C
CCC   REZ1  = DO*DSN    - THETA*HK
      Z1_HK =           - THETA
      Z1_TH = DO*DSN_TH -       HK
      Z1_RT = DO*DSN_RT
      Z1_MS = DO*DSN_MS
C
CCC   REZ2  = DO*THN    - THETA
      Z2_HK = 0.0
      Z2_TH = DO*THN_TH - 1.0
      Z2_RT = DO*THN_RT
      Z2_MS = DO*THN_MS
C
      DO_HK = -(B11*Z1_HK + B12*Z2_HK)
      DO_TH = -(B11*Z1_TH + B12*Z2_TH)
      DO_RT = -(B11*Z1_RT + B12*Z2_RT)
      DO_MS = -(B11*Z1_MS + B12*Z2_MS)
C
      UT_HK = 0.0
      UT_TH = 0.0
      UT_RT = 0.0
      UT_MS = 0.0
C
      BB_HK = 0.0
      BB_TH = 0.0
      BB_RT = 0.0
      BB_MS = 0.0
C
      UT_HK = -(B21*Z1_HK + B22*Z2_HK)
      UT_TH = -(B21*Z1_TH + B22*Z2_TH)
      UT_RT = -(B21*Z1_RT + B22*Z2_RT)
      UT_MS = -(B21*Z1_MS + B22*Z2_MS)
C
cc      BB_HK = -(B21*Z1_HK + B22*Z2_HK)
cc      BB_TH = -(B21*Z1_TH + B22*Z2_TH)
cc      BB_RT = -(B21*Z1_RT + B22*Z2_RT)
cc      BB_MS = -(B21*Z1_MS + B22*Z2_MS)
C
C
C---- set and linearize Cf
      CF    = SGN*2.0*UT**2
      CF_UT = SGN*4.0*UT
      CF_DO = 0.0
C
      CF_HK = CF_UT*UT_HK + CF_DO*DO_HK
      CF_TH = CF_UT*UT_TH + CF_DO*DO_TH
      CF_RT = CF_UT*UT_RT + CF_DO*DO_RT
      CF_MS = CF_UT*UT_MS + CF_DO*DO_MS
C
C
C---- set and linearize "slip" velocity UI = UI( DUO(DO UT TH RT MS) )
      UI    = UO - DUO
      UI_UT =    - DUO_UT
      UI_DO =    - DUO_DO
C
      UI_HK = UI_UT*UT_HK + UI_DO*DO_HK
      UI_TH = UI_UT*UT_TH + UI_DO*DO_TH - DUO_TH
      UI_RT = UI_UT*UT_RT + UI_DO*DO_RT - DUO_RT
      UI_MS = UI_UT*UT_MS + UI_DO*DO_MS - DUO_MS
C

      RETURN
      END ! PRWALL



      SUBROUTINE UWALL(TH,UO,DO,UI,RT,CF,BB, Y,U,N)
C------------------------------------------
C     Returns wall BL profile U(Y).
C
C     Input:
C        TH    kinematic momentum thickness
C        UO    uo/ue outer velocity  (= 1 for normal BL)
C        DO    BL thickness
C        UI    inner "slip" velocity
C        RT    momentum thickness based on ue and THETA
C        CF    wall skin friction
C        BB    outer profile exponent
C        N     number of profile array points
C
C     Output:
C        Y(i)  normal coordinate array
C        U(i)  u/ue velocity profile array
C-------------------------------------------
C
      IMPLICIT REAL (A-H,M,O-Z)
      DIMENSION Y(N), U(N)
      DATA HPI / 1.570796327 /
      DATA AK / 0.09 /
C
C---- log-law constants
      DATA VKAP, VB / 0.40 , 5.0 /
C
      EBK = EXP(-VB*VKAP)
C
      SGN = SIGN( 1.0 , CF )
      UT = SGN * SQRT(0.5*ABS(CF))
C
C
C---- set d+ = DP(UT DO ; RT TH)
      DP    = SGN * UT*RT*DO/TH
C
C---- estimate inner profile edge velocity Ui+ using log-law
      UPE = LOG(DP)/VKAP + VB
C
C---- converge exact Ui+ using Spalding formula
      DO 10 ITUP=1, 5
        UK = UPE*VKAP
        ARG = UK - VB*VKAP
        EXU = EXP(ARG)
        REZ  = UPE +  EXU - EBK*(1.0 + UK + UK*UK/2.0 + UK*UK*UK/6.0)
     &       - DP
        DP_U = 1.0 + (EXU - EBK*(1.0 + UK + UK*UK/2.0))*VKAP
C
        IF(ABS(REZ/DP) .LT. 1.0E-5) GO TO 11
C
        DUPE = -REZ/DP_U
        UPE = UPE + DUPE
 10   CONTINUE
      WRITE(*,*) 'UWALL: Ue+ convergence failed,  Res =', REZ/DP
 11   CONTINUE
C
C          2      2        3      3
C---- set d y+/du+   and  d y+/du+   at BL edge
      DP_UU  = (EXU - EBK*(1.0 + UK))*VKAP**2
      DP_UUU = (EXU - EBK           )*VKAP**3
C
C                         2      2
C---- set  du+/dy+  and  d u+/dy+   at BL edge
      UPD  = 1.0/DP_U
      UPDD = (-1.0/DP_U**3) * DP_UU
C
C---- set coefficients for Spalding profile correction polynomial
      DC2  = 0.5*DP*DP*UPDD    - DP*UPD
      DC3  = -(  DP*DP*UPDD    - DP*UPD   ) / 3.0
C
C---- set outer profile amplitude DUO
      DUO    = UO - UT*(UPE + DC2 + DC3)
C
C
      BB1    =  3.0*(BB    +2.0)*(BB+3.0)/(BB+7.0)
      BB2    = -5.0*(BB    +1.0)*(BB+3.0)/(BB+7.0)
      BB3    =  2.0*(BB    +1.0)*(BB+2.0)/(BB+7.0)
C
c      NE = (N*9)/10
      NE = N
C
C---- fill Y coordinate and U profile arrays
CCC   EXUPE = EXP(UPE*VKAP - VB*VKAP)
      EXUPE = EXU
C
      DEXU = (EXUPE - EBK)/FLOAT(NE-1)
C
      I = 1
      Y(I) = 0.0
      U(I) = 0.0
      DO 20 I=2, NE
ccc     EXU = EBK +  DEXU*FLOAT(I-1)
        EXU = EBK + (DEXU - 0.75*DEXU*FLOAT(NE-I)/FLOAT(NE-1))
     &              *FLOAT(I-1)
C
CCC     UK = UP*VKAP
        UK = LOG(EXU) + VB*VKAP
C
        UP = UK/VKAP
C
C------ set "inverse" Spalding profile  y+(u+)
        YP    =  UP +  EXU - EBK*(1.0 + UK + UK*UK/2.0 + UK*UK*UK/6.0)
        YP_UP = 1.0 + (EXU - EBK*(1.0 + UK + UK*UK/2.0))*VKAP
C
        ET = YP/DP
C
C------ set final inner profile (fudged Spalding)
        UIP = UP + DC2*ET**2 + DC3*ET**3
C
C------ set outer profile
        SQE = SQRT(ET)
        ETB = ET**BB
C
ccc        G = 2.0*ETB - ETB**2
C
        G = (BB1   *ET + BB2   *ET**2 + BB3   *ET**3)*ETB
C
        Y(I) = ET*DO
        U(I) = UT*UIP  +  DUO*G
c
  20  CONTINUE
C
c      DETA = 0.1 / FLOAT(N - NE - 1)
c      DO 300 I=NE+1, N
c        ETA = 1.0 + DETA*FLOAT(I-NE)
c        Y(I) = DO*ETA
c        U(I) = 1.0
c 300  CONTINUE
C
      RETURN
      END ! UWALL



      SUBROUTINE FS(INORM,ISPEC,BSPEC,HSPEC,N,ETAE,GEO,ETA,F,U,S,DELTA)
      IMPLICIT REAL (A-H,M,O-Z)
      DIMENSION ETA(N), F(N), U(N), S(N)
C-----------------------------------------------------
C     Routine for solving the Falkner-Skan equation.
C
C     Input:
C     ------
C      INORM   1: eta = y / sqrt(vx/Ue)  "standard" Falkner-Skan coordinate
C              2: eta = y / sqrt(2vx/(m+1)Ue)  Hartree's coordinate
C              3: eta = y / Theta  momentum thickness normalized coordinate
C      ISPEC   1: BU  = x/Ue dUe/dx ( = "m")  specified
C              2: H12 = Dstar/Theta  specified
C      BSPEC   specified pressure gradient parameter  (if ISPEC = 1)
C      HSPEC   specified shape parameter of U profile (if ISPEC = 2)
C      N       total number of points in profiles
C      ETAE    edge value of normal coordinate
C      GEO     exponential stretching factor for ETA:
C
C     Output:
C     -------
C      BSPEC   calculated pressure gradient parameter  (if ISPEC = 2)
C      HSPEC   calculated shape parameter of U profile (if ISPEC = 1)
C      ETA     normal BL coordinate
C      F,U,S   Falkner Skan profiles
C      DELTA   normal coordinate scale  y = eta * Delta
C-----------------------------------------------------
C
      PARAMETER (NMAX=257,NRMAX=3)
      DIMENSION A(3,3,NMAX),B(3,3,NMAX),C(3,3,NMAX),
     &          R(3,NRMAX,NMAX)
C
C---- set number of righthand sides.
      DATA NRHS / 3 /
C
      ITMAX = 40
C
      IF(N.GT.NMAX) STOP 'FS: Array overflow.'
C
      PI = 4.0*ATAN(1.0)
C
CCC      if(u(n) .ne. 0.0) go to 9991

c
C---- initialize H or BetaU with empirical curve fits
      IF(ISPEC.EQ.1) THEN
       H = 2.6
       BU = BSPEC
      ELSE
       H = HSPEC
       IF(H .LE. 2.17) THEN
        WRITE(*,*) 'FS: Specified H too low'
        H = 2.17
       ENDIF
       BU = (0.058*(H-4.0)**2/(H-1.0) - 0.068) / (6.54*H - 14.07) * H**2
       IF(H .GT. 4.0) BU = MIN( BU , 0.0 )
      ENDIF
C
C---- initialize TN = Delta^2 Ue / vx
      IF(INORM.EQ.3) THEN
       TN = (6.54*H - 14.07) / H**2
      ELSE
       TN = 1.0
      ENDIF
C
C---- set eta array
      DETA = 1.0
      ETA(1) = 0.0
      DO 5 I=2, N
        ETA(I) = ETA(I-1) + DETA
        DETA = GEO*DETA
    5 CONTINUE
C
      DO 6 I=1, N
        ETA(I) = ETA(I) * ETAE/ETA(N)
    6 CONTINUE
C
C
C---- initial guess for profiles using a sine loop for U for half near wall
      IF(H .LE. 3.0) THEN
C
       IF(INORM.EQ.3) THEN
        ETJOIN = 7.3
       ELSE
        ETJOIN = 5.0
       ENDIF
C
       EFAC = 0.5*PI/ETJOIN
       DO 10 I=1, N
         U(I) = SIN(EFAC*ETA(I))
         F(I) = 1.0/EFAC * (1.0 - COS(EFAC*ETA(I)))
         S(I) = EFAC*COS(EFAC*ETA(I))
         IF(ETA(I) .GT. ETJOIN) GO TO 11
  10   CONTINUE
  11   CONTINUE
       IJOIN = I
C
C----- constant U for outer half
       DO 12 I=IJOIN+1, N
         U(I) = 1.0
         F(I) = F(IJOIN) + ETA(I) - ETA(IJOIN)
         S(I) = 0.
   12  CONTINUE
C
      ELSE
C
       IF(INORM.EQ.3) THEN
        ETJOIN1 = 0.0
        ETJOIN2 = 8.0
        IF(H .GT. 4.0) THEN
         ETJOIN1 = H - 4.0
         ETJOIN2 = ETJOIN1 + 8.0
        ENDIF
       ELSE
        ETJOIN1 = 0.0
        ETJOIN2 = 8.0
       ENDIF
C
       DO 13 I=1, N
         U(I) = 0.0
         S(I) = 0.0
         F(I) = 0.0
         IF(ETA(I) .GE. ETJOIN1) GO TO 14
 13    CONTINUE
 14    CONTINUE
       IJOIN = I
C
       EFAC = 0.5*PI/(ETJOIN2-ETJOIN1)
       DO 15 I=IJOIN+1, N
         EBAR = ETA(I) - ETJOIN1
         U(I) = 0.5 - 0.5*COS(2.0*EFAC*EBAR)
         F(I) = 0.5*EBAR - 0.25/EFAC * SIN(2.0*EFAC*EBAR)
         S(I) = EFAC*SIN(2.0*EFAC*EBAR)
         IF(ETA(I) .GE. ETJOIN2) GO TO 16
  15   CONTINUE
  16   CONTINUE
       IJOIN = I
C
C----- constant U for outer half
       DO 17 I=IJOIN+1, N
         U(I) = 1.0
         F(I) = F(IJOIN) + ETA(I) - ETA(IJOIN)
         S(I) = 0.
   17  CONTINUE
C
      ENDIF
c

 9991 continue
C
      RMS = 1.0
C
C---- Newton iteration loop
      DO 100 ITER=1, ITMAX
C
C------ zero out A,B,C blocks and righthand sides R
        DO 20 I=1, N
          DO 201 II=1,3
            DO 2001 III=1,3
              A(II,III,I) = 0.
              B(II,III,I) = 0.
              C(II,III,I) = 0.
 2001       CONTINUE
            R(II,1,I) = 0.
            R(II,2,I) = 0.
            R(II,3,I) = 0.
  201     CONTINUE
   20   CONTINUE
C
C------ calculate Theta in computational space
        THI = 0.
        DO I=1,N-1
          US  = U(I) + U(I+1)
          DETA = ETA(I+1) - ETA(I)
          THI = THI + (1.0 - 0.5*US)*0.5*US*DETA
        ENDDO
        IF(INORM.EQ.3) THEN
c         TN = TN/THI**2
        ENDIF
C
C...................................................
C
        A(1,1,1) = 1.0
        A(2,2,1) = 1.0
        A(3,2,N) = 1.0
        R(1,1,1) = F(1)
        R(2,1,1) = U(1)
        R(3,1,N) = U(N) - 1.0
C
        IF(INORM.EQ.2) THEN
         BETU    = 2.0*BU/(BU+1.0)
         BETU_BU = (2.0 - BETU/(BU+1.0))/(BU+1.0)
         BETN    = 1.0
         BETN_BU = 0.0
        ELSE
         BETU    = BU
         BETU_BU = 1.0
         BETN    = 0.5*(1.0 + BU)
         BETN_BU = 0.5
        ENDIF
C
        DO 30 I=1,N-1
C
          DETA = ETA(I+1) - ETA(I)
          R(1,1,I+1) = F(I+1) - F(I) - 0.5*DETA*(U(I+1)+U(I))
          R(2,1,I+1) = U(I+1) - U(I) - 0.5*DETA*(S(I+1)+S(I))
          R(3,1,I)   = S(I+1) - S(I)
     &      + TN * (  BETN*DETA*0.5*(F(I+1)*S(I+1) + F(I)*S(I))
     &              + BETU*DETA*(1.0 - 0.5*(U(I+1)**2 + U(I)**2)) )
C
          A(3,1,I) =  TN * BETN*0.5*DETA*S(I)
          C(3,1,I) =  TN * BETN*0.5*DETA*S(I+1)
          A(3,2,I) = -TN * BETU    *DETA*U(I)
          C(3,2,I) = -TN * BETU    *DETA*U(I+1)
          A(3,3,I) =  TN * BETN*0.5*DETA*F(I)   - 1.0
          C(3,3,I) =  TN * BETN*0.5*DETA*F(I+1) + 1.0
C
          B(1,1,I+1) = -1.0
          A(1,1,I+1) =  1.0
          B(1,2,I+1) = -0.5*DETA
          A(1,2,I+1) = -0.5*DETA
C
          B(2,2,I+1) = -1.0
          A(2,2,I+1) =  1.0
          B(2,3,I+1) = -0.5*DETA
          A(2,3,I+1) = -0.5*DETA
C
          R(3,2,I) = TN 
     &          * ( BETN_BU*DETA*0.5*(F(I+1)*S(I+1) + F(I)*S(I))
     &            + BETU_BU*DETA*(1.0 - 0.5*(U(I+1)**2 + U(I)**2)))
          R(3,3,I) = ( BETN*DETA*0.5*(F(I+1)*S(I+1) + F(I)*S(I))
     &               + BETU*DETA*(1.0 - 0.5*(U(I+1)**2 + U(I)**2)) )
C
  30    CONTINUE
C...........................................................
C
C
C---- solve Newton system for the three solution vectors
      CALL B3SOLV(A,B,C,R,N,NRHS,NRMAX)
C
C
C---- calculate and linearize Dstar, Theta, in computational space
      DSI = 0.
      DSI1 = 0.
      DSI2 = 0.
      DSI3 = 0.
C
      THI = 0.
      THI1 = 0.
      THI2 = 0.
      THI3 = 0.
C
      DO 40 I=1,N-1
        US  = U(I) + U(I+1)
        DETA = ETA(I+1) - ETA(I)
C
        DSI = DSI + (1.0 - 0.5*US)*DETA
        DSI_US = -0.5*DETA
C
        THI = THI + (1.0 - 0.5*US)*0.5*US*DETA
        THI_US = (0.5 - 0.5*US)*DETA
C
        DSI1 = DSI1 + DSI_US*(R(2,1,I) + R(2,1,I+1))
        DSI2 = DSI2 + DSI_US*(R(2,2,I) + R(2,2,I+1))
        DSI3 = DSI3 + DSI_US*(R(2,3,I) + R(2,3,I+1))
C
        THI1 = THI1 + THI_US*(R(2,1,I) + R(2,1,I+1))
        THI2 = THI2 + THI_US*(R(2,2,I) + R(2,2,I+1))
        THI3 = THI3 + THI_US*(R(2,3,I) + R(2,3,I+1))
  40  CONTINUE
C
C
      IF(ISPEC.EQ.1) THEN
C
C----- set and linearize  Bu = Bspec  residual
       R1 = BSPEC - BU
       Q11 = 1.0
       Q12 = 0.0
C
      ELSE
C
C----- set and linearize  H = Hspec  residual
       R1  =  DSI  - HSPEC*THI
     &       -DSI1 + HSPEC*THI1
       Q11 = -DSI2 + HSPEC*THI2
       Q12 = -DSI3 + HSPEC*THI3
C
      ENDIF
C
C
      IF(INORM.EQ.3) THEN
C
C----- set and linearize  normalized Theta = 1  residual
       R2  =  THI  - 1.0
     &       -THI1
       Q21 = -THI2
       Q22 = -THI3
C
c      R2  =  TN - 0.505
c      IF(ITER.LT.13) THEN
c      R2  =  0.
c      Q21 = 0.0
c      Q22 = 1.0
c      ENDIF

      ELSE
C
C----- set eta scaling coefficient to unity
       R2  =  TN - 1.0
       Q21 = 0.0
       Q22 = 1.0
C
      ENDIF
C
C
      DET =   Q11*Q22 - Q12*Q21
      DBU = -(R1 *Q22 - Q12*R2 ) / DET
      DTN = -(Q11*R2  - R1 *Q21) / DET
C
C
C---- calculate changes in F,U,S, and the max and rms change
      RMAX = 0.
      RMS = 0.
      DO 50 I=1,N
        DF = -R(1,1,I) - DBU*R(1,2,I) - DTN*R(1,3,I)
        DU = -R(2,1,I) - DBU*R(2,2,I) - DTN*R(2,3,I)
        DS = -R(3,1,I) - DBU*R(3,2,I) - DTN*R(3,3,I)
C
        RMAX = MAX(RMAX,ABS(DF),ABS(DU),ABS(DS))
        RMS = DF**2 + DU**2 + DS**2  +  RMS
   50 CONTINUE
      RMS = SQRT(RMS/(3.0*FLOAT(N) + 3.0))
C
      RMAX = MAX(RMAX,ABS(DBU/1.0),ABS(DTN/TN))
C
C---- set underrelaxation factor if necessary by limiting max change to 0.5
      RLX = 1.0
      IF(RMAX.GT.0.5) RLX = 0.5/RMAX
C
C---- update F,U,S
      DO 60 I=1,N
        DF = -R(1,1,I) - DBU*R(1,2,I) - DTN*R(1,3,I)
        DU = -R(2,1,I) - DBU*R(2,2,I) - DTN*R(2,3,I)
        DS = -R(3,1,I) - DBU*R(3,2,I) - DTN*R(3,3,I)
C
        F(I) = F(I) + RLX*DF
        U(I) = U(I) + RLX*DU
        S(I) = S(I) + RLX*DS
   60 CONTINUE
C
C---- update BetaU and Theta
      BU = BU + RLX*DBU
      TN = TN + RLX*DTN
C    
c     write(*,'(1x,i5,e13.5,3f10.5,2x,f8.4)') iter, rms, bu,thi,tn, rlx

C---- check for convergence
      IF(ITER.GT.3 .AND. RMS .LT. 1.0E-6) GO TO 105
C
  100 CONTINUE
      WRITE(*,*) 'FS: Convergence failed'
C
  105 CONTINUE
C
      HSPEC = DSI/THI
      BSPEC = BU
C
      DELTA = SQRT(TN)
C
      RETURN
C
C     The
      END


      SUBROUTINE B3SOLV(A,B,C,R,N,NRHS,NRMAX)
      IMPLICIT REAL (A-H,M,O-Z)
      DIMENSION A(3,3,N), B(3,3,N), C(3,3,N), R(3,NRMAX,N)
C     **********************************************************************
C      This routine solves a 3x3 block-tridiagonal system with an arbitrary
C      number of righthand sides by a standard block elimination scheme.  
C      The solutions are returned in the Rj vectors.
C
C      |A C      ||d|   |R..|
C      |B A C    ||d|   |R..|
C      |  B . .  ||.| = |R..|
C      |    . . C||.|   |R..|
C      |      B A||d|   |R..|
C                                                  Mark Drela   10 March 86
C     **********************************************************************
C
CCC** Forward sweep: Elimination of lower block diagonal (B's).
      DO 1 I=1, N
C
        IM = I-1
C
C------ don't eliminate B1 block because it doesn't exist
        IF(I.EQ.1) GO TO 12
C
C------ eliminate Bi block, thus modifying Ai and Ci blocks
        DO 11 K=1, 3
          DO 111 L=1, 3
            A(K,L,I) = A(K,L,I)
     & - (B(K,1,I)*C(1,L,IM) + B(K,2,I)*C(2,L,IM) + B(K,3,I)*C(3,L,IM))
  111     CONTINUE
          DO 112 L=1, NRHS
            R(K,L,I) = R(K,L,I)
     & - (B(K,1,I)*R(1,L,IM) + B(K,2,I)*R(2,L,IM) + B(K,3,I)*R(3,L,IM))
  112     CONTINUE
   11   CONTINUE
C
C                                                              -1
CCC---- multiply Ci block and righthand side Ri vectors by (Ai)
C       using Gaussian elimination.
C
   12   DO 13 KPIV=1, 2
          KP1 = KPIV+1
C
C-------- find max pivot index KX
          KX = KPIV
          DO 131 K=KP1, 3
            IF(ABS(A(K,KPIV,I)) .LE. ABS(A(KX,KPIV,I))) THEN
             GO TO 131
            ELSE
             GO TO 1311
            ENDIF
 1311       KX = K
  131     CONTINUE
C
          IF(A(KX,KPIV,I).EQ.0.0) THEN
           WRITE(*,*) 'Singular A block, i = ',I
           STOP
          ENDIF
C
          PIVOT = 1.0/A(KX,KPIV,I)
C
C-------- switch pivots
          A(KX,KPIV,I) = A(KPIV,KPIV,I)
C
C-------- switch rows & normalize pivot row
          DO 132 L=KP1, 3
            TEMP = A(KX,L,I)*PIVOT
            A(KX,L,I) = A(KPIV,L,I)
            A(KPIV,L,I) = TEMP
  132     CONTINUE
C
          DO 133 L=1, 3
            TEMP = C(KX,L,I)*PIVOT
            C(KX,L,I) = C(KPIV,L,I)
            C(KPIV,L,I) = TEMP
  133     CONTINUE
C
          DO 134 L=1, NRHS
            TEMP = R(KX,L,I)*PIVOT
            R(KX,L,I) = R(KPIV,L,I)
            R(KPIV,L,I) = TEMP
  134     CONTINUE
CB
C-------- forward eliminate everything
          DO 135 K=KP1, 3
            DO 1351 L=KP1, 3
              A(K,L,I) = A(K,L,I) - A(K,KPIV,I)*A(KPIV,L,I)
 1351       CONTINUE
            C(K,1,I) = C(K,1,I) - A(K,KPIV,I)*C(KPIV,1,I)
            C(K,2,I) = C(K,2,I) - A(K,KPIV,I)*C(KPIV,2,I)
            C(K,3,I) = C(K,3,I) - A(K,KPIV,I)*C(KPIV,3,I)
            DO 1352 L=1, NRHS
              R(K,L,I) = R(K,L,I) - A(K,KPIV,I)*R(KPIV,L,I)
 1352       CONTINUE
  135     CONTINUE
C
   13   CONTINUE
C
C------ solve for last row
        IF(A(3,3,I).EQ.0.0) THEN
         WRITE(*,*) 'Singular A block, i = ',I
         STOP
        ENDIF
        PIVOT = 1.0/A(3,3,I)
        C(3,1,I) = C(3,1,I)*PIVOT
        C(3,2,I) = C(3,2,I)*PIVOT
        C(3,3,I) = C(3,3,I)*PIVOT
        DO 14 L=1, NRHS
          R(3,L,I) = R(3,L,I)*PIVOT
   14   CONTINUE
C
C------ back substitute everything
        DO 15 KPIV=2, 1, -1
          KP1 = KPIV+1
          DO 151 K=KP1, 3
            C(KPIV,1,I) = C(KPIV,1,I) - A(KPIV,K,I)*C(K,1,I)
            C(KPIV,2,I) = C(KPIV,2,I) - A(KPIV,K,I)*C(K,2,I)
            C(KPIV,3,I) = C(KPIV,3,I) - A(KPIV,K,I)*C(K,3,I)
            DO 1511 L=1, NRHS
              R(KPIV,L,I) = R(KPIV,L,I) - A(KPIV,K,I)*R(K,L,I)
 1511       CONTINUE
  151     CONTINUE
   15   CONTINUE
    1 CONTINUE
C
CCC** Backward sweep: Back substitution using upper block diagonal (Ci's).
      DO 2 I=N-1, 1, -1
        IP = I+1
        DO 21 L=1, NRHS
          DO 211 K=1, 3
            R(K,L,I) = R(K,L,I)
     & - (R(1,L,IP)*C(K,1,I) + R(2,L,IP)*C(K,2,I) + R(3,L,IP)*C(K,3,I))
  211     CONTINUE
   21   CONTINUE
    2 CONTINUE
C
      RETURN
      END ! B3SOLV
