
      SUBROUTINE SECONDS(TSEC)
      REAL*8 TSEC
C
      REAL*8 MICSEC
C
C---- get CPU time in microseconds
      CALL CLOCKX(MICSEC)
C
C---- return seconds
      TSEC = MICSEC/1.0D6
C
      RETURN
      END


