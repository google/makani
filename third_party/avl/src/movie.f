
      SUBROUTINE MOVIE(AZIM, ELEV, TILT, RINV, KEIG, IR)
C
C----Plotting module for AVL vortex lattice program
C
C    Plots geometry and loading results for vortex lattice
C     Geometry plots:
C       Surfaces     (straight tapered panels of chordwise strips of vortices)
C       Strips       (chordwise strips of vortices)
C       Vortex legs  (bound vortex legs)
C       Control pts  (vortex element control points)
C       Camber slope (camber of each strip - used for establishing BC's)
C       Hinge lines  (surface deflection axis and deflected surface outline)
C       Strip loading (chordwise plot of vortex loading on each strip)
C
      INCLUDE 'AVL.INC'
      INCLUDE 'AVLPLT.INC'
C
      LOGICAL LKEYS, LEVIEW, LCPAN
      LOGICAL ERROR
      LOGICAL LINITVIEW, LFIRST
      SAVE    LINITVIEW
      CHARACTER*4 OPT
      CHARACTER*1 CHKEY, ANS
C
      REAL ANGE(3), POSE(3)
      REAL ANG(3), POS(3), ANGP(3),DANG(3)
      REAL TT(3,3), TT_ANG(3,3,3),
     &     RT(3,3), RT_ANG(3,3,3)
C
      REAL EVR(JEMAX)
C
      REAL RINP(10)
C
C---- viewpoint changes (deg), zoom/unzoom, perspective scale factors
      DATA DAZIM, DELEV, ZUFAC, PRAT / 5.0 , 5.0 , 1.5 , 1.1 /
C
C---- phase step and scale factor step for interactive phase plots
      DATA DPHASE, SCALEF / 5.0 , 1.25 /
C
C
C---- Initialization for plot program variables
      IF(LPLTNEW .OR. (.NOT.LPLOT)) THEN 
        LPLTNEW = .FALSE.
        LINITVIEW = .FALSE.
      ENDIF
C
      LFIRST = .TRUE.
C
C---- default is camera pans with aircraft
      LCPAN = .TRUE.
C
C---- initial phase and eigenvector scale
      EPHASE = 0.
      EIGENF = 1.
C
C---- find geometry limits
      CALL GLIMS(GMIN,GMAX,.FALSE.)
C
C***************************************************
C---- Setup view transformation 
    4 CALL VIEWINIT(AZIM, ELEV, TILT, RINV)
      CALL VIEWPROJ(UNT,3,ORG)
C
C***************************************************
C
      SIGMA = REAL(EVAL(KEIG,IR))
      OMEGA = IMAG(EVAL(KEIG,IR))
      EVMAG = ABS( EVAL(KEIG,IR) )
C
      DAMP = -SIGMA / EVMAG
C
C---- set reasonable plot time interval, cycle it only once
      TPLOT = 1.0 / MAX( SLOMOF*ABS(SIGMA)/(2.0*PI) , 
     &                   SLOMOF*ABS(OMEGA)/(8.0*PI) ,
     &                   1.0/TMOVIE )
C
      write(*,*) 's', ABS(SIGMA)/(2.0*PI)
      write(*,*) 'w', ABS(OMEGA)/(8.0*PI)
      write(*,*) 'T', 1.0/TMOVIE
C
c 53     RINPUT(1) = TMOVIE
c        WRITE(*,1163) RINPUT(1), UNCHT(1:NUT)
c 1163   FORMAT(/' Enter play time of movie:', F10.3,' (',A,')')
c        CALL READR(1,RINPUT,ERROR)
c        IF(ERROR) GO TO 53
c        TMOVIE = RINPUT(1)

C-----------------------------------------------------------
C---- make sure we don't get trapped in very long movie
      REALT = TPLOT/SLOMOF
      IF(REALT .GT. 20.0) THEN
       WRITE(*,*) 'Movie will require real time =', REALT
       WRITE(*,*) 'Continue with new slo-mo factor?  Y'
       READ (*,1000) ANS
 1000  FORMAT(A)
       IF(INDEX('Nn',ANS).NE.0) RETURN
C
       WRITE(*,*) 'Enter slow-motion factor (bigger = faster):', SLOMOF
       CALL READR(1,SLOMOF,ERROR)
      ENDIF
C-----------------------------------------------------------
C



C***************************************************
C---- compute perturbed position at current phase angle
    6 CONTINUE
C
      VEE = PARVAL(IPVEE,IR)
      REFL = BREF*UNITL
      REFV = VEE
C
      EVMIN = (REFV/REFL) * 1.0E-5
C
      SIGMA = REAL(EVAL)
      OMEGA = IMAG(EVAL)
      EVMAG = SQRT(SIGMA**2 + OMEGA**2)
C
      CALL EVNORM(EVEC,ESF,REFL,REFV)
      EFAC = ESF*EIGENF
C


C---- set time using specified phase
      IF(EVMAG .LT. EVMIN) THEN
       TIMED = EPHASE*DTR / EVMIN
C
      ELSE
       TIMED = EPHASE*DTR / MAX( ABS(OMEGA) , ABS(SIGMA) )
C
      ENDIF
C


      CALL EVREAL(EVEC(1,KEIG,IR),EVAL(KEIG,IR), EFAC,TIMED, EVR)
C

C
C---- scale from standard (SI or English) to AVL units
      TIME = TIMED*VEE/UNITL
C
      EVR(JEX) = EVR(JEX)/UNITL
      EVR(JEY) = EVR(JEY)/UNITL
      EVR(JEZ) = EVR(JEZ)/UNITL
C
      EVR(JEU) = EVR(JEU)/VEE
      EVR(JEV) = EVR(JEV)/VEE
      EVR(JEW) = EVR(JEW)/VEE
C
      EVR(JEP) = EVR(JEP)*UNITL/VEE
      EVR(JEQ) = EVR(JEQ)*UNITL/VEE
      EVR(JER) = EVR(JER)*UNITL/VEE
C
CCC   EVR(JEPH) = EVR(JEPH)
CCC   EVR(JETH) = EVR(JETH)
CCC   EVR(JEPS) = EVR(JEPS)
C
C
C
c      write(*,*)
c      write(*,*) 'xyz', evr(jex),evr(jey),evr(jez)
c      write(*,*) 'uvw', evr(jeu),evr(jev),evr(jew)
c      write(*,*) 'pqr', evr(jep),evr(jeq),evr(jer)
c      write(*,*) 'reh', evr(jeph),evr(jeth),evr(jeps)

C
C---- set perturbed Earth-coordinate position
      IF(LCPAN) THEN
C----- camera panning... no baseline movement
       POS(1) = 0.
       POS(2) = 0.
       POS(3) = 0.
C
       ANG(1) = PARVAL(IPPHI,IR)*DTR
       ANG(2) = PARVAL(IPTHE,IR)*DTR
       ANG(3) = PARVAL(IPPSI,IR)*DTR
C
      ELSE
C----- camera is fixed... include aicraft motion, integrated up to present time
C
C----- sat baseline velocities and rotation rates
       ALFA = PARVAL(IPALFA,IR)*DTR
       BETA = PARVAL(IPBETA,IR)*DTR
       CALL VINFAB
       WROT(1) = PARVAL(IPROTX,IR)*2.0/BREF
       WROT(2) = PARVAL(IPROTY,IR)*2.0/CREF
       WROT(3) = PARVAL(IPROTZ,IR)*2.0/BREF
C
C----- set time step based on max Euler angle change
       RMAX = MAX( ABS(WROT(1)) , ABS(WROT(2)) , ABS(WROT(3)) )
       DAMAX = RMAX*ABS(TIME)
       NTIME = INT( DAMAX / 0.025 )
C
C----- set initial position,angles at t=0
       POS(1) = 0.
       POS(2) = 0.
       POS(3) = 0.
       ANG(1) = PARVAL(IPPHI,IR)*DTR
       ANG(2) = PARVAL(IPTHE,IR)*DTR
       ANG(3) = PARVAL(IPPSI,IR)*DTR
       CALL RATEKI3(ANG,RT,RT_ANG)
C
C----- integrate over time interval  t = 0..TIME
       DO ITIME = 1, NTIME
         DT = TIME/FLOAT(NTIME)
C
C------- predictor step, slopes evaluated at  t
         DO K = 1, 3
           DANG(K) = DT*( RT(K,1)*WROT(1)
     &                  + RT(K,2)*WROT(2)
     &                  + RT(K,3)*WROT(3) )
           ANGP(K) = ANG(K) + DANG(K)
         ENDDO
         CALL RATEKI3(ANGP,RT,RT_ANG)
C
C------- corrector step, slopes evaluated at  t + dt
         DO K = 1, 3
           DANGP = DT*( RT(K,1)*WROT(1)
     &                + RT(K,2)*WROT(2)
     &                + RT(K,3)*WROT(3) )
C--------- midpoint angles at  t + dt/2
           ANGP(K) = ANG(K) + 0.25*(DANG(K) + DANGP)
         ENDDO
C
C------- use midpoint-angle matrices
         CALL RATEKI3(ANGP,RT,RT_ANG)
         CALL ROTENS3(ANGP,TT,TT_ANG)
C
C------- final integration step, using midpoint slopes
         DO K = 1, 3
           POS(K) = POS(K) - DT*( TT(K,1)*VINF(1)
     &                          + TT(K,2)*VINF(2)
     &                          + TT(K,3)*VINF(3) )
           ANG(K) = ANG(K) + DT*( RT(K,1)*WROT(1)
     &                          + RT(K,2)*WROT(2)
     &                          + RT(K,3)*WROT(3) )
         ENDDO
       ENDDO
C
      ENDIF
C
C---- set final position, including eigenvector perturbation
      POSE(1) = POS(1) + EVR(JEX)
      POSE(2) = POS(2) + EVR(JEY)
      POSE(3) = POS(3) + EVR(JEZ)
C
C---- set final angles, including eigenvector perturbation
      ANGE(1) = ANG(1) + EVR(JEPH)
      ANGE(2) = ANG(2) + EVR(JETH)
      ANGE(3) = ANG(3) + EVR(JEPS)
C
C
c      CALL RATEKI3(ANG,RT,RT_ANG)
c      DO K = 1, 3
c        DANG(K) =    RT(K,1)*WROT(1)
c     &             + RT(K,2)*WROT(2)
c     &             + RT(K,3)*WROT(3)
c      ENDDO
c      write(1,1234) ( pos(k)/12.0, k=1, 3),
c     &              ( ang(k)/dtr , k=1, 3),
c     &              ( dang(k)*bref/dtr    , k=1, 3)
c      write(2,1234) (pose(k)/12.0, k=1, 3), (ange(k)/dtr, k=1, 3)
c 1234 format(1x,9f10.3)
C
C
C
      XYZREF(1) = PARVAL(IPXCG,IR)
      XYZREF(2) = PARVAL(IPYCG,IR)
      XYZREF(3) = PARVAL(IPZCG,IR)
C
C---- set limits for baseline position
      CALL ROTENS3(ANG,TT,TT_ANG)
      CALL GRLIMS(VMINP,VMAXP,.TRUE. ,TT ,XYZREF,POS )
      IF(LFIRST) THEN
C----- first frame... set plot limits directly
       XMIN = VMINP(1)
       YMIN = VMINP(2)
       XMAX = VMAXP(1)
       YMAX = VMAXP(2)
      ELSE
C----- clip to new limits
       XMIN = MIN(XMIN,VMINP(1))
       YMIN = MIN(YMIN,VMINP(2))
       XMAX = MAX(XMAX,VMAXP(1))
       YMAX = MAX(YMAX,VMAXP(2))
      ENDIF
      LFIRST = .FALSE.
C
C
C---- also enforce limits for perturbed position
      CALL ROTENS3(ANGE,TT,TT_ANG)
      CALL GRLIMS(VMINP,VMAXP,.TRUE. ,TT ,XYZREF,POSE)
      XMIN = MIN(XMIN,VMINP(1))
      YMIN = MIN(YMIN,VMINP(2))
      XMAX = MAX(XMAX,VMAXP(1))
      YMAX = MAX(YMAX,VMAXP(2))
C
      CALL OFFINI
C
C***************************************************
C---- plot the baseline and displaced geometry
    8 CONTINUE
      CALL PVLINI(TITLE,AZIM,ELEV,TILT,VERSION,LSVMOV)
C
      CALL GETWINSIZE(XWIND,YWIND)
C
      CCH = 0.8*CH
      XPLT = XABS2USR(PMARG)
      YPLT = YABS2USR(YWIND-PMARG) - 1.2*CCH
C
      CALL PLCHAR(XPLT,YPLT,CCH,'Run  ',0.0,5)
      CALL PLNUMB(999.,YPLT,CCH,FLOAT(IR),0.0,-1)
C
      YPLT = YPLT - 2.2*CCH
      CALL PLCHAR(XPLT,YPLT,CCH,'Mode ',0.0,5)
      CALL PLNUMB(999.,YPLT,CCH,FLOAT(KEIG),0.0,-1)
C
      YPLT = YPLT - 2.2*CCH
      FRQ = OMEGA / (2.0*PI)
      CALL PLCHAR(XPLT,YPLT,CCH,'f = ',0.0,4)
      CALL PLNUMB(999.,YPLT,CCH,FRQ,0.0,4)
      CALL PLCHAR(999.,YPLT,CCH,' cycles/' ,0.0,8)
      CALL PLCHAR(999.,YPLT,CCH,UNCHT(1:NUT),0.0,NUT)
C
      YPLT = YPLT - 2.2*CCH
      IF(SIGMA .EQ. 0.0) THEN
       DAMPR = 0.
      ELSE
       DAMPR = -SIGMA / SQRT(SIGMA**2 + OMEGA**2)
      ENDIF
      CALL PLMATH(XPLT,YPLT,CCH,'z = ',0.0,4)
      CALL PLNUMB(999.,YPLT,CCH,DAMPR,0.0,6)
C
      YPLT = YPLT - 2.2*CCH
      CALL PLMATH(XPLT,YPLT,CCH,'f = ',0.0, 4)
      CALL PLNUMB(999.,YPLT,CCH,EPHASE,0.0,-1)
      CALL PLMATH(999.,YPLT,CCH,   '"',0.0, 1)
C
C---- Setup hidden line data
      CALL HIDINITE(.TRUE., ANGE,POSE,XYZREF)
C
      CALL PLOTMODE(ANG ,POS ,XYZREF,0)
      CALL PLOTMODE(ANGE,POSE,XYZREF,1)
      CALL PLFLUSH
C
ccc      CALL DRAWTOSCREEN



