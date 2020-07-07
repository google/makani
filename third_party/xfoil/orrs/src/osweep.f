      PROGRAM OSWEEP
      LOGICAL OK
C
      WRITE(*,*) 'Enter  Rth1, w1, H1'
      READ (*,*) RSP1, WSP1, HSP1
      IF(RSP1.EQ.0.0) STOP
C
      WRITE(*,*) 'Enter  Rth2, w2, H2'
      READ (*,*) RSP2, WSP2, HSP2
      IF(RSP2.EQ.0.0) STOP
C
      RLSP1 = LOG10(RSP1)
      RLSP2 = LOG10(RSP2)
C
      WLSP1 = LOG10(WSP1)
      WLSP2 = LOG10(WSP2)
C
      HLSP1 = HSP1
      HLSP2 = HSP2
C
C
      KK = 1000
C
      LU = 1
      WRITE(LU,1000)
 1000 FORMAT(
     & '#    Rtheta     w Theta/Ue        H       ',
     &  '    ar Theta      ai Theta' )
CCC      1234567890123|1234567890123|1234567890123|1234567890123|1234567890123|
      DO K = 0, KK
        T = FLOAT(K)/FLOAT(KK)
C
        RL = RLSP1*(1.0-T) + RLSP2*T
        WL = WLSP1*(1.0-T) + WLSP2*T
        HL = HLSP1*(1.0-T) + HLSP2*T
C
        R = 10.0 ** RL
        W = 10.0 ** WL
        H = HL
C
        CALL OSMAP(R,W,H,
     &             AR,
     &             AR_R, AR_W, AR_H,
     &             ARW_R,ARW_W,ARW_H,
     &             AI,
     &             AI_R, AI_W, AI_H,
     &             AIW_R,AIW_W,AIW_H, OK )
        WRITE(1,1200) R, W, H, AR, AI, 
     &               AR_R, AR_W, AR_H, 
     &               AI_R, AI_W, AI_H
 1200   FORMAT(1X, 16E14.6)
      ENDDO
      STOP
C
      END
