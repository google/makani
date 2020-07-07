C*********************************************************************** 
C    Module:  mousexy.f
C 
C    Copyright (C) 2012 Harold Youngren, Mark Drela 
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
C
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C*********************************************************************** 


      program mousexy
c---------------------------------------------------------------
c     Interactive mouse and continuous input test program.
c
c     Lets user specify points in a spline, then allows the user to 
c     drag spline points around to change the curve
c---------------------------------------------------------------
c
      parameter (ipx=100)
c
      character*1 cin
c
      common /Rpoints/
     &       ch,
     &       x1(ipx), y1(ipx), s1(ipx), xp1(ipx), yp1(ipx),
     &       x2(ipx), y2(ipx), s2(ipx), xp2(ipx), yp2(ipx)
      common /Ipoints/
     &       iplt, n1, n2
      logical lok
c
      ch = 0.13
      n1 = 0
      n2 = 0
      iplt = 0
c
 1000 format(a)
C
C---Initialize the plot package before we get into plotting...
      CALL PLINITIALIZE

C---put up plot window and refresh
      call pltall('(terminate with q or double point)')

c
ccc      call NEWFACTOR(6.0)
c
 5    write(*,1050) 
 1050 format(/' Enter points for spline (q or ret to end)')

C--- Get initial set of points from user
      call NEWCOLORNAME('green')
      call NEWPEN(1)
C
      xlast = -999.
      ylast = -999.
      ii = 0
      do j = 1, 100
        call GETCURSORXY(xx,yy,ikey)
        cin = char(ikey)
        if(cin.EQ.'q' .OR. cin.EQ.'Q') go to 100
C--- check for doubled point to end input
          if( (xx-xlast).EQ.0.0 .AND. 
     &        (yy-ylast).EQ.0.0 ) go to 100 
        ii = ii + 1
        x1(ii) = xx
        y1(ii) = yy
         if(ii.EQ.1) then
          call plot(xx,yy,3)
          CALL PLSYMB(999.,999.,0.4*CH,2,0.0,0)
         else
          call plot(xlast,ylast,3)
          call plot(xx,yy,2)
          CALL PLSYMB(999.,999.,0.4*CH,2,0.0,0)
          call PLFLUSH
        endif
        xlast = xx
        ylast = yy
      end do
C
 100  n1 = ii
C
      call plend
C--- initialize modified point arrays 
       do j = 1, n1
         x2(j) = x1(j)      
         y2(j) = y1(j)
       end do
       n2 = n1

      call pltall('Select point to modify ("i" to reset)')
c
      write(*,*)
      write(*,*) 'Move spline points...'
C
 200  call GETCURSORXY(xx,yy,ikey)
cc      call NEWCOLORNAME('green')
cc      call plot(xx,yy,3)
cc      CALL PLSYMB(999.,999.,0.4*CH,2,0.0,0)
      cin = char(ikey)
      write(*,*) 'ikey cin ',ikey,cin

      if(cin.EQ.'q' .OR. cin.EQ.'Q') go to 400
C--- reset modified point arrays 
      if(cin.EQ.'i' .OR. cin.EQ.'I') then
       write(*,*) 're-initializing points...'
       do j = 1, n1
         x2(j) = x1(j)      
         y2(j) = y1(j)
       end do
       n2 = n1
       call pltall('Select point to modify ("i" to reset)')
       go to 200
      endif

C--- find closest point to cursor     
      jp = 0
      dsqmin = 999.
      do j = 1, n1
        dsq = (xx-x2(j))**2 + (yy-y2(j))**2
        if(dsq.LT.dsqmin) then
          dsqmin = dsq
          jp = j
        endif
      end do
c
      write(*,*) 'Cursor at x,y ',xx,yy
      write(*,*) 'Point ',jp,' selected at x,y ',x1(jp),y1(jp) 
C--- now read cursor and move point
 300  call GETCURSORXYC(xx,yy,ibtn)
      write(*,*) xx,yy,ibtn
      if(ibtn.eq.0) go to 200
      x2(jp) = xx
      y2(jp) = yy
      n2 = n1
      call pltall('Move point...("i" to reset)')
      go to 300      
c
 400  continue
ccc      pause
      stop
      end

      subroutine pltall(msg)
      character*(*) msg
      parameter (ipx=100)
c
      common /Rpoints/
     &       ch,
     &       x1(ipx), y1(ipx), s1(ipx), xp1(ipx), yp1(ipx),
     &       x2(ipx), y2(ipx), s2(ipx), xp2(ipx), yp2(ipx)
      common /Ipoints/
     &       iplt, n1, n2
C
      call PLOPEN(0.8,0,1)
      CALL PLCHAR(0.5,0.5,1.2*CH,'TEST FOR MOUSE READ',0.0,-1)
      CALL PLCHAR(0.3,0.3,1.2*CH,msg,0.0,-1)
ccc      call PLOT(5.5, 4.25, -3)

      if(n1.GT.1) then
C--- spline input curve and plot
      call scalc(x1,y1,s1,n1)
      call splind(x1,xp1,s1,n1,-999.,-999.)        
      call splind(y1,yp1,s1,n1,-999.,-999.)        
C
      call NEWCOLORNAME('blue')
      call plot(x1(1),y1(1),3)
      CALL PLSYMB(999.,999.,0.4*CH,1,0.0,0)
      ninter = 10
      dfn = 1.0/float(ninter)
      do j = 1, n1-1
        ds = s1(j+1) - s1(j)
        do n = 1, ninter
          ss = s1(j) + ds*float(n)*dfn
          xx = seval(ss,x1,xp1,s1,n1)
          yy = seval(ss,y1,yp1,s1,n1)
          call plot(xx,yy,2)
        end do
        CALL PLSYMB(999.,999.,0.4*CH,1,0.0,0)
      end do
      endif

      if(n2.GT.1) then
C--- spline modified curve and plot
      call scalc(x2,y2,s2,n2)
      call splind(x2,xp2,s2,n2,-999.,-999.)        
      call splind(y2,yp2,s2,n2,-999.,-999.)        
C
      call NEWCOLORNAME('red')
      call plot(x2(1),y2(1),3)
      CALL PLSYMB(999.,999.,0.4*CH,1,0.0,0)
      ninter = 10
      dfn = 1.0/float(ninter)
      do j = 1, n2-1
        ds = s2(j+1) - s2(j)
        do n = 1, ninter
          ss = s2(j) + ds*float(n)*dfn
          xx = seval(ss,x2,xp2,s2,n2)
          yy = seval(ss,y2,yp2,s2,n2)
          call plot(xx,yy,2)
        end do
        CALL PLSYMB(999.,999.,0.4*CH,1,0.0,0)
      end do
      endif

      call PLFLUSH
      return
      end

C***********************************************************************
C    Module:  spline.f
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

      SUBROUTINE SPLINE(X,XS,S,N)
      DIMENSION X(N),XS(N),S(N)
      PARAMETER (NMAX=1000)
      DIMENSION A(NMAX),B(NMAX),C(NMAX)
C-------------------------------------------------------
C     Calculates spline coefficients for X(S).          |
C     Zero 2nd derivative end conditions are used.      |
C     To evaluate the spline at some value of S,        |
C     use SEVAL and/or DEVAL.                           |
C                                                       |
C     S        independent variable array (input)       |
C     X        dependent variable array   (input)       |
C     XS       dX/dS array                (calculated)  |
C     N        number of points           (input)       |
C                                                       |
C-------------------------------------------------------
      IF(N.GT.NMAX) STOP 'SPLINE: array overflow, increase NMAX'
C     
      DO 1 I=2, N-1
        DSM = S(I) - S(I-1)
        DSP = S(I+1) - S(I)
        B(I) = DSP
        A(I) = 2.0*(DSM+DSP)
        C(I) = DSM
        XS(I) = 3.0*((X(I+1)-X(I))*DSM/DSP + (X(I)-X(I-1))*DSP/DSM)
    1 CONTINUE
C
C---- set zero second derivative end conditions
      A(1) = 2.0
      C(1) = 1.0
      XS(1) = 3.0*(X(2)-X(1)) / (S(2)-S(1))
      B(N) = 1.0
      A(N) = 2.0
      XS(N) = 3.0*(X(N)-X(N-1)) / (S(N)-S(N-1))
C
C---- solve for derivative array XS
      CALL TRISOL(A,B,C,XS,N)
C
      RETURN
      END ! SPLINE      


      SUBROUTINE SPLIND(X,XS,S,N,XS1,XS2)
      DIMENSION X(N),XS(N),S(N)
      PARAMETER (NMAX=1000)
      DIMENSION  A(NMAX),B(NMAX),C(NMAX)
C-------------------------------------------------------
C     Calculates spline coefficients for X(S).          |
C     Specified 1st derivative and/or usual zero 2nd    |
C     derivative end conditions are used.               |
C     To evaluate the spline at some value of S,        |
C     use SEVAL and/or DEVAL.                           |
C                                                       |
C     S        independent variable array (input)       |
C     X        dependent variable array   (input)       |
C     XS       dX/dS array                (calculated)  |
C     N        number of points           (input)       |
C     XS1,XS2  endpoint derivatives       (input)       |
C              If = 999.0, then usual zero second       |
C              derivative end condition(s) are used     |
C              If = -999.0, then zero third             |
C              derivative end condition(s) are used     |
C                                                       |
C-------------------------------------------------------
      IF(N.GT.NMAX) STOP 'SPLIND: array overflow, increase NMAX'
C     
      DO 1 I=2, N-1
        DSM = S(I) - S(I-1)
        DSP = S(I+1) - S(I)
        B(I) = DSP
        A(I) = 2.0*(DSM+DSP)
        C(I) = DSM
        XS(I) = 3.0*((X(I+1)-X(I))*DSM/DSP + (X(I)-X(I-1))*DSP/DSM)
    1 CONTINUE
C
      IF(XS1.EQ.999.0) THEN
C----- set zero second derivative end condition
       A(1) = 2.0
       C(1) = 1.0
       XS(1) = 3.0*(X(2)-X(1)) / (S(2)-S(1))
      ELSE IF(XS1.EQ.-999.0) THEN
C----- set zero third derivative end condition
       A(1) = 1.0
       C(1) = 1.0
       XS(1) = 2.0*(X(2)-X(1)) / (S(2)-S(1))
      ELSE
C----- set specified first derivative end condition
       A(1) = 1.0
       C(1) = 0.
       XS(1) = XS1
      ENDIF
C
      IF(XS2.EQ.999.0) THEN
       B(N) = 1.0
       A(N) = 2.0
       XS(N) = 3.0*(X(N)-X(N-1)) / (S(N)-S(N-1))
      ELSE IF(XS2.EQ.-999.0) THEN
       B(N) = 1.0
       A(N) = 1.0
       XS(N) = 2.0*(X(N)-X(N-1)) / (S(N)-S(N-1))
      ELSE
       A(N) = 1.0
       B(N) = 0.
       XS(N) = XS2
      ENDIF
C
      IF(N.EQ.2 .AND. XS1.EQ.-999.0 .AND. XS2.EQ.-999.0) THEN
       B(N) = 1.0
       A(N) = 2.0
       XS(N) = 3.0*(X(N)-X(N-1)) / (S(N)-S(N-1))
      ENDIF
C
C---- solve for derivative array XS
      CALL TRISOL(A,B,C,XS,N)
C
      RETURN
      END ! SPLIND

 

      SUBROUTINE SPLINA(X,XS,S,N)
      IMPLICIT REAL (A-H,O-Z)
      DIMENSION X(N),XS(N),S(N)
      LOGICAL LEND
C-------------------------------------------------------
C     Calculates spline coefficients for X(S).          |
C     A simple averaging of adjacent segment slopes     |
C     is used to achieve non-oscillatory curve          |
C     End conditions are set by end segment slope       |
C     To evaluate the spline at some value of S,        |
C     use SEVAL and/or DEVAL.                           |
C                                                       |
C     S        independent variable array (input)       |
C     X        dependent variable array   (input)       |
C     XS       dX/dS array                (calculated)  |
C     N        number of points           (input)       |
C                                                       |
C-------------------------------------------------------
C     
      LEND = .TRUE.
      DO 1 I=1, N-1
        DS = S(I+1)-S(I)
        IF (DS.EQ.0.) THEN
          XS(I) = XS1
          LEND = .TRUE.
         ELSE
          DX = X(I+1)-X(I)
          XS2 = DX / DS
          IF (LEND) THEN
            XS(I) = XS2
            LEND = .FALSE.
           ELSE
            XS(I) = 0.5*(XS1 + XS2)
          ENDIF
        ENDIF
        XS1 = XS2
    1 CONTINUE
      XS(N) = XS1
C
      RETURN
      END ! SPLINA



      SUBROUTINE TRISOL(A,B,C,D,KK)
      DIMENSION A(KK),B(KK),C(KK),D(KK)
C-----------------------------------------
C     Solves KK long, tri-diagonal system |
C                                         |
C             A C          D              |
C             B A C        D              |
C               B A .      .              |
C                 . . C    .              |
C                   B A    D              |
C                                         |
C     The righthand side D is replaced by |
C     the solution.  A, C are destroyed.  |
C-----------------------------------------
C
      DO 1 K=2, KK
        KM = K-1
        C(KM) = C(KM) / A(KM)
        D(KM) = D(KM) / A(KM)
        A(K) = A(K) - B(K)*C(KM)
        D(K) = D(K) - B(K)*D(KM)
    1 CONTINUE
C
      D(KK) = D(KK)/A(KK)
C
      DO 2 K=KK-1, 1, -1
        D(K) = D(K) - C(K)*D(K+1)
    2 CONTINUE
C
      RETURN
      END ! TRISOL


      FUNCTION SEVAL(SS,X,XS,S,N)
      DIMENSION X(N), XS(N), S(N)
C--------------------------------------------------
C     Calculates X(SS)                             |
C     XS array must have been calculated by SPLINE |
C--------------------------------------------------
      ILOW = 1
      I = N
C
   10 IF(I-ILOW .LE. 1) GO TO 11
C
      IMID = (I+ILOW)/2
      IF(SS .LT. S(IMID)) THEN
       I = IMID
      ELSE
       ILOW = IMID
      ENDIF
      GO TO 10
C
   11 DS = S(I) - S(I-1)
      T = (SS - S(I-1)) / DS
      CX1 = DS*XS(I-1) - X(I) + X(I-1)
      CX2 = DS*XS(I)   - X(I) + X(I-1)
      SEVAL = T*X(I) + (1.0-T)*X(I-1) + (T-T*T)*((1.0-T)*CX1 - T*CX2)
      RETURN
      END ! SEVAL

      FUNCTION DEVAL(SS,X,XS,S,N)
      DIMENSION X(N), XS(N), S(N)
C--------------------------------------------------
C     Calculates dX/dS(SS)                         |
C     XS array must have been calculated by SPLINE |
C--------------------------------------------------
      ILOW = 1
      I = N
C
   10 IF(I-ILOW .LE. 1) GO TO 11
C
      IMID = (I+ILOW)/2
      IF(SS .LT. S(IMID)) THEN
       I = IMID
      ELSE
       ILOW = IMID
      ENDIF
      GO TO 10
C
   11 DS = S(I) - S(I-1)
      T = (SS - S(I-1)) / DS
      CX1 = DS*XS(I-1) - X(I) + X(I-1)
      CX2 = DS*XS(I)   - X(I) + X(I-1)
      DEVAL = X(I) - X(I-1) + (1.-4.0*T+3.0*T*T)*CX1 + T*(3.0*T-2.)*CX2
      DEVAL = DEVAL/DS
      RETURN
      END ! DEVAL

      FUNCTION D2VAL(SS,X,XS,S,N)
      DIMENSION X(N), XS(N), S(N)
C--------------------------------------------------
C     Calculates d2X/dS2(SS)                       |
C     XS array must have been calculated by SPLINE |
C--------------------------------------------------
      ILOW = 1
      I = N
C
   10 IF(I-ILOW .LE. 1) GO TO 11
C
      IMID = (I+ILOW)/2
      IF(SS .LT. S(IMID)) THEN
       I = IMID
      ELSE
       ILOW = IMID
      ENDIF
      GO TO 10
C
   11 DS = S(I) - S(I-1)
      T = (SS - S(I-1)) / DS
      CX1 = DS*XS(I-1) - X(I) + X(I-1)
      CX2 = DS*XS(I)   - X(I) + X(I-1)
      D2VAL = (6.*T-4.)*CX1 + (6.*T-2.0)*CX2
      D2VAL = D2VAL/DS**2
      RETURN
      END ! D2VAL


      FUNCTION CURV(SS,X,XS,Y,YS,S,N)
      DIMENSION X(N), XS(N), Y(N), YS(N), S(N)
C-----------------------------------------------
C     Calculates curvature of splined 2-D curve |
C     at S = SS                                 |
C                                               |
C     S        arc length array of curve        |
C     X, Y     coordinate arrays of curve       |
C     XS,YS    derivative arrays                |
C              (calculated earlier by SPLINE)   |
C-----------------------------------------------
C     
      ILOW = 1
      I = N
C
   10 IF(I-ILOW .LE. 1) GO TO 11
C
      IMID = (I+ILOW)/2
      IF(SS .LT. S(IMID)) THEN
       I = IMID
      ELSE
       ILOW = IMID
      ENDIF
      GO TO 10
C
   11 DS = S(I) - S(I-1)
      T = (SS - S(I-1)) / DS
C
      CX1 = DS*XS(I-1) - X(I) + X(I-1)
      CX2 = DS*XS(I)   - X(I) + X(I-1)
      XD = X(I) - X(I-1) + (1.0-4.0*T+3.0*T*T)*CX1 + T*(3.0*T-2.0)*CX2
      XDD = (6.0*T-4.0)*CX1 + (6.0*T-2.0)*CX2
C
      CY1 = DS*YS(I-1) - Y(I) + Y(I-1)
      CY2 = DS*YS(I)   - Y(I) + Y(I-1)
      YD = Y(I) - Y(I-1) + (1.0-4.0*T+3.0*T*T)*CY1 + T*(3.0*T-2.0)*CY2
      YDD = (6.0*T-4.0)*CY1 + (6.0*T-2.0)*CY2
C 
      SD = SQRT(XD*XD + YD*YD)
      SD = MAX(SD,0.001*DS)
C
      CURV = (XD*YDD - YD*XDD) / SD**3
C
      RETURN
      END ! CURV


      FUNCTION CURVS(SS,X,XS,Y,YS,S,N)
      DIMENSION X(N), XS(N), Y(N), YS(N), S(N)
C-----------------------------------------------
C     Calculates curvature derivative of        |
C     splined 2-D curve at S = SS               |
C                                               |
C     S        arc length array of curve        |
C     X, Y     coordinate arrays of curve       |
C     XS,YS    derivative arrays                |
C              (calculated earlier by SPLINE)   |
C-----------------------------------------------
C     
      ILOW = 1
      I = N
C
   10 IF(I-ILOW .LE. 1) GO TO 11
C
      IMID = (I+ILOW)/2
      IF(SS .LT. S(IMID)) THEN
       I = IMID
      ELSE
       ILOW = IMID
      ENDIF
      GO TO 10
C
   11 DS = S(I) - S(I-1)
      T = (SS - S(I-1)) / DS
C
      CX1 = DS*XS(I-1) - X(I) + X(I-1)
      CX2 = DS*XS(I)   - X(I) + X(I-1)
      XD = X(I) - X(I-1) + (1.0-4.0*T+3.0*T*T)*CX1 + T*(3.0*T-2.0)*CX2
      XDD = (6.0*T-4.0)*CX1 + (6.0*T-2.0)*CX2
      XDDD = 6.0*CX1 + 6.0*CX2
C
      CY1 = DS*YS(I-1) - Y(I) + Y(I-1)
      CY2 = DS*YS(I)   - Y(I) + Y(I-1)
      YD = Y(I) - Y(I-1) + (1.0-4.0*T+3.0*T*T)*CY1 + T*(3.0*T-2.0)*CY2
      YDD = (6.0*T-4.0)*CY1 + (6.0*T-2.0)*CY2
      YDDD = 6.0*CY1 + 6.0*CY2
C
      SD = SQRT(XD*XD + YD*YD)
      SD = MAX(SD,0.001*DS)
C
      BOT = SD**3
      DBOTDT = 3.0*SD*(XD*XDD + YD*YDD)
C
      TOP = XD*YDD - YD*XDD      
      DTOPDT = XD*YDDD - YD*XDDD
C
      CURVS = (DTOPDT*BOT - DBOTDT*TOP) / BOT**2
C
      RETURN
      END ! CURVS


      SUBROUTINE SINVRT(SI,XI,X,XS,S,N)
      DIMENSION X(N), XS(N), S(N)
C-------------------------------------------------------
C     Calculates the "inverse" spline function S(X).    |
C     Since S(X) can be multi-valued or not defined,    |
C     this is not a "black-box" routine.  The calling   |
C     program must pass via SI a sufficiently good      |
C     initial guess for S(XI).                          |
C                                                       |
C     XI      specified X value       (input)           |
C     SI      calculated S(XI) value  (input,output)    |
C     X,XS,S  usual spline arrays     (input)           |
C                                                       |
C-------------------------------------------------------
C
      SISAV = SI
C
      DO 10 ITER=1, 10
        RES  = SEVAL(SI,X,XS,S,N) - XI
        RESP = DEVAL(SI,X,XS,S,N)
        DS = -RES/RESP
        SI = SI + DS
        IF(ABS(DS/(S(N)-S(1))) .LT. 1.0E-5) RETURN
   10 CONTINUE
      WRITE(*,*)
     &  'SINVRT: spline inversion failed. Input value returned.'
      SI = SISAV
C
      RETURN
      END ! SINVRT


      SUBROUTINE SCALC(X,Y,S,N)
      DIMENSION X(N), Y(N), S(N)
C----------------------------------------
C     Calculates the arc length array S  |
C     for a 2-D array of points (X,Y).   |
C----------------------------------------
C
      S(1) = 0.
      DO 10 I=2, N
        S(I) = S(I-1) + SQRT((X(I)-X(I-1))**2 + (Y(I)-Y(I-1))**2)
   10 CONTINUE
C
      RETURN
      END ! SCALC


      SUBROUTINE SPLNXY(X,XS,Y,YS,S,N)
      DIMENSION X(N), XS(N), Y(N), YS(N), S(N)
C-----------------------------------------
C     Splines 2-D shape X(S), Y(S), along |
C     with true arc length parameter S.   |
C-----------------------------------------
      PARAMETER (KMAX=32)
      DIMENSION XT(0:KMAX), YT(0:KMAX)
C
      KK = KMAX
      NPASS = 10
C
C---- set first estimate of arc length parameter
      CALL SCALC(X,Y,S,N)
C
C---- spline X(S) and Y(S)
      CALL SEGSPL(X,XS,S,N)
      CALL SEGSPL(Y,YS,S,N)
C
C---- re-integrate true arc length
      DO 100 IPASS=1, NPASS
C
        SERR = 0.
C
        DS = S(2) - S(1)
        DO I = 2, N
          DX = X(I) - X(I-1)
          DY = Y(I) - Y(I-1)
C
          CX1 = DS*XS(I-1) - DX
          CX2 = DS*XS(I  ) - DX
          CY1 = DS*YS(I-1) - DY
          CY2 = DS*YS(I  ) - DY
C
          XT(0) = 0.
          YT(0) = 0.
          DO K=1, KK-1
            T = FLOAT(K) / FLOAT(KK)
            XT(K) = T*DX + (T-T*T)*((1.0-T)*CX1 - T*CX2)
            YT(K) = T*DY + (T-T*T)*((1.0-T)*CY1 - T*CY2)
          ENDDO
          XT(KK) = DX
          YT(KK) = DY
C
          SINT1 = 0.
          DO K=1, KK
            SINT1 = SINT1
     &            + SQRT((XT(K)-XT(K-1))**2 + (YT(K)-YT(K-1))**2)
          ENDDO
C
          SINT2 = 0.
          DO K=2, KK, 2
            SINT2 = SINT2
     &            + SQRT((XT(K)-XT(K-2))**2 + (YT(K)-YT(K-2))**2)
          ENDDO
C
          SINT = (4.0*SINT1 - SINT2) / 3.0
C
          IF(ABS(SINT-DS) .GT. ABS(SERR))  SERR = SINT - DS
C
          IF(I.LT.N) DS = S(I+1) - S(I)
C
          S(I) = S(I-1) + SQRT(SINT)
        ENDDO
C
        SERR = SERR / (S(N) - S(1))
        WRITE(*,*) IPASS, SERR
C
C------ re-spline X(S) and Y(S)
        CALL SEGSPL(X,XS,S,N)
        CALL SEGSPL(Y,YS,S,N)
C
        IF(ABS(SERR) .LT. 1.0E-7) RETURN
C
 100  CONTINUE
C
      RETURN
      END ! SPLNXY



      SUBROUTINE SEGSPL(X,XS,S,N)
C-----------------------------------------------
C     Splines X(S) array just like SPLINE,      |
C     but allows derivative discontinuities     |
C     at segment joints.  Segment joints are    |
C     defined by identical successive S values. |
C-----------------------------------------------
      DIMENSION X(N), XS(N), S(N)
C
      IF(S(1).EQ.S(2)  ) STOP 'SEGSPL:  First input point duplicated'
      IF(S(N).EQ.S(N-1)) STOP 'SEGSPL:  Last  input point duplicated'
C
      ISEG0 = 1
      DO 10 ISEG=2, N-2
        IF(S(ISEG).EQ.S(ISEG+1)) THEN
         NSEG = ISEG - ISEG0 + 1
         CALL SPLIND(X(ISEG0),XS(ISEG0),S(ISEG0),NSEG,-999.0,-999.0)
         ISEG0 = ISEG+1
        ENDIF
   10 CONTINUE
C
      NSEG = N - ISEG0 + 1
      CALL SPLIND(X(ISEG0),XS(ISEG0),S(ISEG0),NSEG,-999.0,-999.0)
C
      RETURN
      END ! SEGSPL



      SUBROUTINE SEGSPLD(X,XS,S,N,XS1,XS2)
C-----------------------------------------------
C     Splines X(S) array just like SPLIND,      |
C     but allows derivative discontinuities     |
C     at segment joints.  Segment joints are    |
C     defined by identical successive S values. |
C-----------------------------------------------
      DIMENSION X(N), XS(N), S(N)
C
      IF(S(1).EQ.S(2)  ) STOP 'SEGSPL:  First input point duplicated'
      IF(S(N).EQ.S(N-1)) STOP 'SEGSPL:  Last  input point duplicated'
C
      ISEG0 = 1
      DO 10 ISEG=2, N-2
        IF(S(ISEG).EQ.S(ISEG+1)) THEN
         NSEG = ISEG - ISEG0 + 1
         CALL SPLIND(X(ISEG0),XS(ISEG0),S(ISEG0),NSEG,XS1,XS2)
         ISEG0 = ISEG+1
        ENDIF
   10 CONTINUE
C
      NSEG = N - ISEG0 + 1
      CALL SPLIND(X(ISEG0),XS(ISEG0),S(ISEG0),NSEG,XS1,XS2)
C
      RETURN
      END ! SEGSPL

