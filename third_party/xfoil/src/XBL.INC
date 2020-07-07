C
      PARAMETER (NCOM=73)
      REAL COM1(NCOM), COM2(NCOM)
      REAL M1, M1_U1, M1_MS, M2, M2_U2, M2_MS
      LOGICAL SIMI,TRAN,TURB,WAKE
      LOGICAL TRFORC,TRFREE
C
C-    SCCON  =  shear coefficient lag constant
C-    GACON  =  G-beta locus constants...
C-    GBCON  =   G = GACON * sqrt(1.0 + GBCON*beta) 
C-    GCCON  =         + GCCON / [H*Rtheta*sqrt(Cf/2)]   <-- wall term
C-    DLCON  =  wall/wake dissipation length ratio  Lo/L
C-    CTCON  =  Ctau weighting coefficient (implied by G-beta constants)
C
      INCLUDE 'BLPAR.INC'
C
      COMMON/V_VAR1/ X1,  U1,  T1,  D1,  S1, AMPL1, U1_UEI, U1_MS, DW1
     &        , H1, H1_T1, H1_D1
     &        , M1, M1_U1,                            M1_MS
     &        , R1, R1_U1,                            R1_MS
     &        , V1, V1_U1,                            V1_MS,  V1_RE
     &        , HK1, HK1_U1, HK1_T1, HK1_D1,         HK1_MS
     &        , HS1, HS1_U1, HS1_T1, HS1_D1,         HS1_MS, HS1_RE
     &        , HC1, HC1_U1, HC1_T1, HC1_D1,         HC1_MS
     &        , RT1, RT1_U1, RT1_T1,                 RT1_MS, RT1_RE
     &        , CF1, CF1_U1, CF1_T1, CF1_D1,         CF1_MS, CF1_RE
     &        , DI1, DI1_U1, DI1_T1, DI1_D1, DI1_S1, DI1_MS, DI1_RE
     &        , US1, US1_U1, US1_T1, US1_D1,         US1_MS, US1_RE
     &        , CQ1, CQ1_U1, CQ1_T1, CQ1_D1,         CQ1_MS, CQ1_RE
     &        , DE1, DE1_U1, DE1_T1, DE1_D1,         DE1_MS
      COMMON/V_VAR2/ X2,  U2,  T2,  D2,  S2, AMPL2, U2_UEI, U2_MS, DW2
     &        , H2, H2_T2, H2_D2
     &        , M2, M2_U2,                            M2_MS
     &        , R2, R2_U2,                            R2_MS
     &        , V2, V2_U2,                            V2_MS,  V2_RE
     &        , HK2, HK2_U2, HK2_T2, HK2_D2,         HK2_MS
     &        , HS2, HS2_U2, HS2_T2, HS2_D2,         HS2_MS, HS2_RE
     &        , HC2, HC2_U2, HC2_T2, HC2_D2,         HC2_MS
     &        , RT2, RT2_U2, RT2_T2,                 RT2_MS, RT2_RE
     &        , CF2, CF2_U2, CF2_T2, CF2_D2,         CF2_MS, CF2_RE
     &        , DI2, DI2_U2, DI2_T2, DI2_D2, DI2_S2, DI2_MS, DI2_RE
     &        , US2, US2_U2, US2_T2, US2_D2,         US2_MS, US2_RE
     &        , CQ2, CQ2_U2, CQ2_T2, CQ2_D2,         CQ2_MS, CQ2_RE
     &        , DE2, DE2_U2, DE2_T2, DE2_D2,         DE2_MS
      EQUIVALENCE (X1,COM1(1)), (X2,COM2(1))
C
      COMMON/V_VARA/ CFM, CFM_MS, CFM_RE
     &        , CFM_U1, CFM_T1, CFM_D1
     &        , CFM_U2, CFM_T2, CFM_D2
     &        , XT,    XT_A1, XT_MS, XT_RE, XT_XF
     &        , XT_X1, XT_T1, XT_D1, XT_U1
     &        , XT_X2, XT_T2, XT_D2, XT_U2
C
C
      COMMON/V_SAV/ C1SAV(NCOM), C2SAV(NCOM)
C
      COMMON/V_VAR/ DWTE
     &          , QINFBL
     &          , TKBL  , TKBL_MS
     &          , RSTBL , RSTBL_MS
     &          , HSTINV, HSTINV_MS
     &          , REYBL , REYBL_MS, REYBL_RE
     &          , GAMBL, GM1BL, HVRAT
     &          , BULE, XIFORC, AMCRIT
C
      COMMON/V_INT/ SIMI,TRAN,TURB,WAKE
     &          , TRFORC,TRFREE
     &          , IDAMPV
C
      COMMON/V_SYS/ VS1(4,5),VS2(4,5),VSREZ(4),VSR(4),VSM(4),VSX(4)
C

