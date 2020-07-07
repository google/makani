      PROGRAM OTEST
      REAL AI(-1:1), AI_R(-1:1),  AI_W(-1:1),  AI_H(-1:1),
     &              AIW_R(-1:1), AIW_W(-1:1), AIW_H(-1:1)
      REAL AR(-1:1), AR_R(-1:1),  AR_W(-1:1),  AR_H(-1:1),
     &              ARW_R(-1:1), ARW_W(-1:1), ARW_H(-1:1)
      LOGICAL OK
C
    1 WRITE(*,*) 'Enter Rth, w, H'
      READ (*,*) RSP, WSP, HSP
      IF(RSP.EQ.0.0) STOP
C
      WRITE(*,*) 'Enter dRth, dw, dH'
      READ (*,*) DR, DW, DH
C
      DO I=-1, 1
cc      I = 0
        R = RSP + DR*FLOAT(I)
        CALL OSMAP(R,WSP,HSP,
     &             AI(I),
     &             AI_R(I), AI_W(I), AI_H(I),
     &             AIW_R(I),AIW_W(I),AIW_H(I),
     &             AR(I),
     &             AR_R(I), AR_W(I), AR_H(I),
     &             ARW_R(I),ARW_W(I),ARW_H(I), OK )
      ENDDO
      WRITE(*,*) 'ai   :', AI(0)
      DADR = (AI(1) - AI(-1))*0.5/DR
      WRITE(*,*) 'da/dR:', DADR, AI_R(-1), AI_R(0), AI_R(1)
C
      DO I=-1, 1
cc      I = 0
        H = HSP + DH*FLOAT(I)
        CALL OSMAP(RSP,WSP,H,
     &             AI(I),
     &             AI_R(I), AI_W(I), AI_H(I),
     &             AIW_R(I),AIW_W(I),AIW_H(I),
     &             AR(I),
     &             AR_R(I), AR_W(I), AR_H(I),
     &             ARW_R(I),ARW_W(I),ARW_H(I), OK )
      ENDDO
      DADH = (AI(1) - AI(-1))*0.5/DH
      WRITE(*,*) 'da/dH:', DADH, AI_H(-1), AI_H(0), AI_H(1)
C
      DO I=-1, 1
cc      I = 0
        W = WSP + DW*FLOAT(I)
        CALL OSMAP(RSP,W,HSP,
     &             AI(I),
     &             AI_R(I), AI_W(I), AI_H(I),
     &             AIW_R(I),AIW_W(I),AIW_H(I),
     &             AR(I),
     &             AR_R(I), AR_W(I), AR_H(I),
     &             ARW_R(I),ARW_W(I),ARW_H(I), OK )
      ENDDO
      DADW = (AI(1) - AI(-1))*0.5/DW
      WRITE(*,*) 'da/dw:', DADW, AI_W(-1), AI_W(0), AI_W(1)
C
      DBDR = (AI_R(1) - AI_R(-1))*0.5/DW
      WRITE(*,*) 'daR/dw:', DBDR, AIW_R(-1), AIW_R(0), AIW_R(1)
      DBDH = (AI_H(1) - AI_H(-1))*0.5/DW
      WRITE(*,*) 'daH/dw:', DBDH, AIW_H(-1), AIW_H(0), AIW_H(1)
      DBDW = (AI_W(1) - AI_W(-1))*0.5/DW
      WRITE(*,*) 'daw/dw:', DBDW, AIW_W(-1), AIW_W(0), AIW_W(1)
C
C
      GO TO 1
      END
