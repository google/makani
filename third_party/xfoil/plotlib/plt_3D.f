C***********************************************************************
C    Module:  plt_3D.f
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
C***********************************************************************
C --- Xplot11 3D routines
C
C     Version 4.46 11/28/01
C
C     Note:  These are routine(s) that provide some means of displaying 
C            3D objects in conjunction with the usual XPlot11 routines.  
C            They are by no means complete but can serve as a starting
C            point for doing simple 3D graphics.
C
C***********************************************************************


      subroutine VIEW(X,Y,Z,N,XP,YP,XOB,YOB,ZOB,ROBINV,XUP,YUP,ZUP)
      DIMENSION X(N), Y(N), Z(N)
      DIMENSION XP(N), YP(N)
C........................................................................
C
C        Projects one or more points in 3-D Cartesian space
C     onto a 2-D plane from the viewpoint of an observer
C     at a specified location.  This can be used to view 
C     a 3-D object (described by a set of x,y,z points) 
C     by projecting the points into a set of x,y points for
C     plotting on a planar 2-D graphics screen.
C        The viewing  plane, which has its own x,y coordinate
C     system, always faces the observer but can be turned
C     around the viewing axis, thus simulating the observer
C     tilting his head while looking at the object.  This tilt
C     is specified by giving a vector in x,y,z space which 
C     "points up" relative to the observer.
C        The distance of the observer from the object is specified
C     explicitly.  This does not affect much the size of the viewed
C     object, since the viewing plane contains the 3-D space origin
C     and hence is at or near the object.  It does however affect the
C     apparent distortion of the object due to perspective.  This
C     is very useful to convey the 3-dimensionality of the object.
C     If the observer is very very far away, there is no distortion
C     (as in a mechanical drawing).
C
C     X,Y,Z        Cartesian point coordinates                   (input)
C     N            number of points                              (input)
C     XP,YP        projected point coordinates on viewing plane  (output)
C     XOB,YOB,ZOB  Cartesian vector pointing towards observer    (input)
C                  (magnitude irrelevant)
C     ROBINV       1/(distance to observer)                      (input)
C     XUP,YUP,ZUP  Cartesian vector which points "up" from the 
C                  observer's viewpoint (magnitude irrelevant)   (input)
C
C                                        Mark Drela     July 1988
C........................................................................
C
C---- unit view vector perpendicular to viewing plane (towards observer)
      XOBN = XOB/SQRT(XOB**2 + YOB**2 + ZOB**2)
      YOBN = YOB/SQRT(XOB**2 + YOB**2 + ZOB**2)
      ZOBN = ZOB/SQRT(XOB**2 + YOB**2 + ZOB**2)
C
C---- vector along plane's local x coordinate: (up vector)X(view vector)
      XIP = YUP*ZOBN - ZUP*YOBN
      YIP = ZUP*XOBN - XUP*ZOBN
      ZIP = XUP*YOBN - YUP*XOBN
C
C---- normalize plane's x coordinate vector
      XIHAT = XIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
      YIHAT = YIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
      ZIHAT = ZIP/SQRT(XIP**2 + YIP**2 + ZIP**2)
C
C---- unit vector along plane's y coordinate: (view vector)X(x unit vector)
      XJHAT = YOBN*ZIHAT - ZOBN*YIHAT
      YJHAT = ZOBN*XIHAT - XOBN*ZIHAT
      ZJHAT = XOBN*YIHAT - YOBN*XIHAT
C
C---- go over all points
      DO 10 I=1, N
C
        RDOTR = X(I)*XOBN + Y(I)*YOBN + Z(I)*ZOBN
C
C------ viewing-axis component of vector
        DRX = RDOTR*XOBN
        DRY = RDOTR*YOBN
        DRZ = RDOTR*ZOBN
C
C------ projected vector scaling factor due to perspective
        VSCAL = 1.0 / SQRT( (XOBN-ROBINV*DRX)**2
     &                    + (YOBN-ROBINV*DRY)**2
     &                    + (ZOBN-ROBINV*DRZ)**2 )
C
C------ dot vector into plane coordinate system unit vectors, and scale
        XP(I) = (XIHAT*X(I) + YIHAT*Y(I) + ZIHAT*Z(I))*VSCAL
        YP(I) = (XJHAT*X(I) + YJHAT*Y(I) + ZJHAT*Z(I))*VSCAL
C
   10 CONTINUE
C
      RETURN
      END


      subroutine VIEWR(R,N,RP,ROB,ROBINV,RUP)
      DIMENSION R(3,N)
      DIMENSION RP(3,N)
      DIMENSION ROB(3), RUP(3)
C........................................................................
C
C     Same as VIEW, but vectors are passed in R(1:3,...) array form.
C     The out-of-plane RP(3...) value is also returned.
C
C     R(..)    Cartesian point coordinates                   (input)
C     N        number of points                              (input)
C     RP(..)   projected point coordinates on viewing plane  (output)
C     ROB(.)   Cartesian vector pointing towards observer    (input)
C                  (magnitude irrelevant)
C     ROBINV   1/(distance to observer)                      (input)
C     RUP(.)   Cartesian vector which points "up" from the 
C              observer's viewpoint (magnitude irrelevant)   (input)
C
C                                        Mark Drela     July 2007
C........................................................................
      REAL RIP(3), RIN(3), RJN(3), RKN(3), DR(3), RI(3)
C
C---- unit view vector perpendicular to viewing plane (towards observer)
      SOB = SQRT(ROB(1)**2 + ROB(2)**2 + ROB(3)**2)
      RKN(1) = ROB(1)/SOB
      RKN(2) = ROB(2)/SOB
      RKN(3) = ROB(3)/SOB
C
C---- vector along plane's local x coordinate: (up vector)X(view vector)
      RIP(1) = RUP(2)*RKN(3) - RUP(3)*RKN(2)
      RIP(2) = RUP(3)*RKN(1) - RUP(1)*RKN(3)
      RIP(3) = RUP(1)*RKN(2) - RUP(2)*RKN(1)
C
C---- normalize plane's x coordinate vector
      SIP = SQRT(RIP(1)**2 + RIP(2)**2 + RIP(3)**2)
      RIN(1) = RIP(1)/SIP
      RIN(2) = RIP(2)/SIP
      RIN(3) = RIP(3)/SIP
C
C---- unit vector along plane's y coordinate: (view vector)X(x unit vector)
      RJN(1) = RKN(2)*RIN(3) - RKN(3)*RIN(2)
      RJN(2) = RKN(3)*RIN(1) - RKN(1)*RIN(3)
      RJN(3) = RKN(1)*RIN(2) - RKN(2)*RIN(1)
C
C---- go over all points
      DO 10 I=1, N
        RI(1) = R(1,I)
        RI(2) = R(2,I)
        RI(3) = R(3,I)
        RDOTR = RI(1)*RKN(1) + RI(2)*RKN(2) + RI(3)*RKN(3)
C
C------ viewing-axis component of vector
        DR(1) = RDOTR*RKN(1)
        DR(2) = RDOTR*RKN(2)
        DR(3) = RDOTR*RKN(3)
C
C------ projected vector scaling factor due to perspective
        VSCAL = 1.0 / SQRT( (RKN(1)-ROBINV*DR(1))**2
     &                    + (RKN(2)-ROBINV*DR(2))**2
     &                    + (RKN(3)-ROBINV*DR(3))**2 )
C
C------ dot vector into plane coordinate system unit vectors, and scale
        RP(1,I) = (RIN(1)*RI(1) + RIN(2)*RI(2) + RIN(3)*RI(3))*VSCAL
        RP(2,I) = (RJN(1)*RI(1) + RJN(2)*RI(2) + RJN(3)*RI(3))*VSCAL
        RP(3,I) = (RKN(1)*RI(1) + RKN(2)*RI(2) + RKN(3)*RI(3))*VSCAL
   10 CONTINUE
C
      RETURN
      END ! VIEWR



      SUBROUTINE PROJMATRIX3 (ROTZ,ROTY,RMAT)
C...Purpose:  To define rotation and transformation matrix. The input 
C             pair of angles ROTZ,ROTY specify the viewpoint by
C             an angle about the Z axis (CCW) and an angle about 
C             the newly rotated Y axis (CCW).  Both angles are 
C             right-handed in a conventional sense about each axis.
C
C...Input:    ROTZ  rotation of viewpoint about     Z axis (deg)
C             ROTY  rotation of viewpoint about new Y axis (deg)
C
C...Output:   RMAT  3x3 rotation and perspective matrix
C
      DIMENSION   RMAT(3,3)
C
      DTR = 4.0*ATAN(1.0)/180.0
      COSZ = COS(ROTZ*DTR)
      SINZ = SIN(ROTZ*DTR)
      COSY = COS(ROTY*DTR)
      SINY = SIN(ROTY*DTR)
C
C---Rotation matrix (rotation about Z, then rotation about Y)
c     xx = -(     SINZ*X +      COSZ*Y)
      RMAT(1,1) = -SINZ
      RMAT(2,1) = -COSZ
      RMAT(3,1) =  0.0
C     yy =   SINY*COSZ*X - SINY*SINZ*Y + COSY*Z
      RMAT(1,2) =  SINY*COSZ
      RMAT(2,2) = -SINY*SINZ
      RMAT(3,2) =  COSY
c     zz = -(COSY*COSZ*X - COSY*SINZ*Y - SINY*Z)
      RMAT(1,3) = -COSY*COSZ
      RMAT(2,3) =  COSY*SINZ
      RMAT(3,3) =  SINY
C
c      xx = -(     SINZ*X +      COSZ*Y)
c      yy =   SINY*COSZ*X - SINY*SINZ*Y + COSY*Z
c      zz = -(COSY*COSZ*X - COSY*SINZ*Y - SINY*Z)
C
c      write(*,*) 'Rmatrix row1 ', (RMAT(1,L),L=1,3)
c      write(*,*) 'Rmatrix row2 ', (RMAT(2,L),L=1,3)
c      write(*,*) 'Rmatrix row3 ', (RMAT(3,L),L=1,3)
c      read(*,*)
C
      RETURN
      END


      SUBROUTINE PROJMATRIX4 (ROTZ,ROTY,RDIST,RMAT)
C...Purpose:  To define rotation and perspective matrix. The input 
C             pair of angles ROTZ,ROTY specify the viewpoint by
C             an angle about the Z axis (CCW) and an angle about 
C             the newly rotated Y axis (CCW).  Both angles are 
C             right-handed in a conventional sense about each axis.
C             The observer distance RDIST specifies the distance from 
C             the origin to the observer along the viewpoint direction.   
C
C...Input:    ROTZ  rotation of viewpoint about     Z axis (deg)
C             ROTY  rotation of viewpoint about new Y axis (deg)
C             RDIST distance from origin to observer along viewpoint
C
C...Output:   RMAT  4x4 rotation and perspective matrix
C
      DIMENSION  AMAT(4,4),PMAT(4,4), RMAT(4,4)
C
      DTR = 4.0*ATAN(1.0)/180.0
      COSZ = COS(ROTZ*DTR)
      SINZ = SIN(ROTZ*DTR)
      COSY = COS(ROTY*DTR)
      SINY = SIN(ROTY*DTR)
C
C---Rotation matrix (rotation about Z, then rotation about Y)
c     xx = -(     SINZ*X +      COSZ*Y)
      AMAT(1,1) = -SINZ
      AMAT(2,1) = -COSZ
      AMAT(3,1) =  0.0
      AMAT(4,1) =  0.0
C     yy =   SINY*COSZ*X - SINY*SINZ*Y + COSY*Z
      AMAT(1,2) =  SINY*COSZ
      AMAT(2,2) = -SINY*SINZ
      AMAT(3,2) =  COSY
      AMAT(4,2) =  0.0
c     zz = -(COSY*COSZ*X - COSY*SINZ*Y - SINY*Z)
      AMAT(1,3) = -COSY*COSZ
      AMAT(2,3) =  COSY*SINZ
      AMAT(3,3) =  SINY
      AMAT(4,3) =  0.0
C
      AMAT(1,4) =  0.0
      AMAT(2,4) =  0.0
      AMAT(3,4) =  0.0
      AMAT(4,4) =  1.0
C
c      xx = -(     SINZ*X +      COSZ*Y)
c      yy =   SINY*COSZ*X - SINY*SINZ*Y + COSY*Z
c      zz = -(COSY*COSZ*X - COSY*SINZ*Y - SINY*Z)
c
C---Perspective matrix with projection on Z plane
      PMAT(1,1) =  1.0
      PMAT(2,1) =  0.0
      PMAT(3,1) =  0.0
      PMAT(4,1) =  0.0
C
      PMAT(1,2) =  0.0
      PMAT(2,2) =  1.0
      PMAT(3,2) =  0.0
      PMAT(4,2) =  0.0
C
      PMAT(1,3) =  0.0
      PMAT(2,3) =  0.0
      PMAT(3,3) =  1.0
      PMAT(4,3) =  0.0
C
      PMAT(1,4) =  0.0
      PMAT(2,4) =  0.0
      PMAT(3,4) = -RDIST
      PMAT(4,4) =  1.0
C
C---Product of matrices is perspective matrix
      DO J=1, 4
        DO K=1, 4
          TMP = 0.0
          DO L=1, 4
            TMP = TMP + AMAT(J,L)*PMAT(L,K)
          END DO
          RMAT(J,K) = TMP
        END DO
      END DO
C
c      write(*,*) 'Rmatrix row1 ', (RMAT(1,L),L=1,4)
c      write(*,*) 'Rmatrix row2 ', (RMAT(2,L),L=1,4)
c      write(*,*) 'Rmatrix row3 ', (RMAT(3,L),L=1,4)
c      write(*,*) 'Rmatrix row4 ', (RMAT(4,L),L=1,4)
c      read(*,*)
C
      RETURN
      END


      SUBROUTINE ROTPTS3 (RMAT,PTS_IN,NPTS,PTS_OUT)
C...Purpose:    To rotate array of points to a new viewpoint by 
C               parallel projection. The input rotation matrix 
C               contains the transformation data in a 3x3 matrix.
C
C...Input:    RMAT     3x3 transformation matrix
C             PTS_IN   array (3xNPTS) of input points 
C             NPTS     number of points in arrays
C
C...Output:   PTS_OUT  array (3xNPTS) of transformed points 
C
      DIMENSION  PTS_IN(3,NPTS), PTS_OUT(3,NPTS)
      DIMENSION  RMAT(4,4)
C
      DO I = 1, NPTS
C
         DO J=1, 3
           TMP = 0.0
           DO K=1, 3
             TMP = TMP + PTS_IN(K,I)*RMAT(K,J)
           END DO
           PTS_OUT(J,I) = TMP
         END DO
C
      END DO
C
      RETURN
      END


      SUBROUTINE ROTPTS4 (RMAT,PTS_IN,NPTS,PTS_OUT)
C...Purpose:    To rotate array of points to a new viewpoint by 
C               perspective projection. The input rotation matrix 
C               contains the transformation and perspective data in 
C               a 4x4 matrix in homogeneous coordinates. Note that input 
C               coordinates may need to be z-clipped if the user trans.
C               moves the points behind the observer. A check is made 
C               for a singular perspective point (at observer).  
C
C...Input:    RMAT  4x4 rotation and perspective matrix
C             PTS_IN   array (3xNPTS) of input points 
C             NPTS     number of points in arrays
C
C...Output:   PTS_OUT  array (3xNPTS) of transformed points 
C
C...Note:     You may need to translate your points to recenter them
C             about the origin to get good perspective views.  Points 
C             off to the side get pretty distorted...
C
      DIMENSION  PTS_IN(3,NPTS), PTS_OUT(3,NPTS)
      DIMENSION  RMAT(4,4), PTMP(4), PPTMP(4)
C
      DO I = 1, NPTS
C
         PTMP(1) = PTS_IN(1,I)
         PTMP(2) = PTS_IN(2,I)
         PTMP(3) = PTS_IN(3,I)
         PTMP(4) = 1.0
C
         DO J=1, 4
           TMP = 0.0
           DO K=1, 4
             TMP = TMP + PTMP(K)*RMAT(K,J)
           END DO
           PPTMP(J) = TMP
         END DO
C
         IF(PPTMP(4).NE.0.0) THEN
           PTS_OUT(1,I) = PPTMP(1)/PPTMP(4)
           PTS_OUT(2,I) = PPTMP(2)/PPTMP(4)
           PTS_OUT(3,I) = PPTMP(3)/PPTMP(4)
          ELSE
           WRITE(*,*) 'Homogeneous coordinate singular for pt ',I
         ENDIF
C
      END DO
C
      RETURN
      END
