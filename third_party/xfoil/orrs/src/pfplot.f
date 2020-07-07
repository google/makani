      PROGRAM PFPLOT
      PARAMETER (NMAX=256)
      REAL ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      CHARACTER*1 ANS
      LOGICAL OK
C
      IDEV = 1
      IDEVRP = 2
      SIZE = 5.0
      IPSLU = 0
      SCRNFR = 0.85
C
      CALL PLINITIALIZE
C
      CALL PLOPEN(SCRNFR,IPSLU,IDEV)
      CALL NEWFACTOR(SIZE)
C
      CALL PLOT(0.7,0.1,-3)
C
      N = 256
      ETAE = 16.0
      GEO = 1.01
C
      EWT = 1.0/ETAE
      UWT = 0.5
      PWT = 0.2
      PWT = 1.0
      CH = 0.02
C
      IF(N.GT.NMAX) STOP 'TEST: Array overflow.'
C
    2 CALL PFLGET(N,GEO,ETAE,ETA,F,U,S,H)
C
      CALL NEWPEN(1)
C
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(UWT*1.0,0.0,2)
      CALL PLOT(0.0,0.0,3)
      CALL PLOT(0.0,EWT*ETAE,2)
C
      CALL NEWPEN(3)
      CALL PLOT(UWT*U(1),EWT*ETA(1),3)
      DO 10 I=2, N
        CALL PLOT(UWT*U(I),EWT*ETA(I),2)
   10 CONTINUE
C
      CALL PLSYMB(UWT       ,EWT*ETA(N)+0.5*CH,CH,'H = ',0.0,4)
      CALL PLNUMB(UWT+4.0*CH,EWT*ETA(N)+0.5*CH,CH, H    ,0.0,3)
      CALL PLFLUSH
C
      CALL ASKL('Another profile ?^',OK)
      IF(OK) GO TO 2
C
      CALL PLCLOSE
      STOP
      END



      SUBROUTINE PFLGET(N,GEO,ETAE,ETA,F,U,S,H)
      REAL ETA(N),F(N),U(N),S(N)
      CHARACTER*48 FNAME
C
C---- eta coordinate normalized with momentum thickness
      INORM = 3
C
      WRITE(6,*) ' '
      WRITE(6,*) '  1   Falkner-Skan parameter m = x/U dU/dx'
      WRITE(6,*) '  2   Falkner-Skan parameter beta = 2m/(m+1)'
      WRITE(6,*) '  3   Falkner-Skan shape parameter H'
      WRITE(6,*) '  4   General profile input file'
      WRITE(6,*) ' '
      CALL ASKI('Select profile option^',IOPT)
C
      IF(IOPT.NE.4) THEN
       CALL ASKI('Enter number of BL points^',N)
       CALL ASKR('Enter geometric stretching factor^',GEO)
       CALL ASKR('Enter edge y/theta value^',ETAE)
      ENDIF
C
C
      IF(IOPT.EQ.1) THEN
C
       CALL ASKR('Enter m^',BU)
       CALL FS(INORM,1,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ELSE IF(IOPT.EQ.2) THEN
C
       CALL ASKR('Enter beta^',BETA)
       BU = BETA/(2.0-BETA)
       CALL FS(INORM,1,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ELSE IF(IOPT.EQ.3) THEN
C
       CALL ASKR('Enter H^',H)
       CALL FS(INORM,2,BU,H,N,ETAE,GEO,ETA,F,U,S,DELTA)
C
      ELSE
C
       CALL ASKS('Enter profile filename^',FNAME)
       OPEN(1,FILE=FNAME,STATUS='OLD')
       READ(1,*) N, H
       DO 5 I=1, N
         READ(1,*) ETA(I), U(I), S(I)
    5  CONTINUE
       CLOSE(1)
C
       GEO = (ETA(3)-ETA(2)) / (ETA(2)-ETA(1))
       ETAE = ETA(N)
      ENDIF
C
      WRITE(6,1050) N, H, ETA(N), GEO
 1050 FORMAT(/' n =', I4,'   H =', F7.3,
     &                   '   Ye =', F7.3,
     &                   '   dYi+1/dYi =',F6.3 /)
C
      RETURN
      END
