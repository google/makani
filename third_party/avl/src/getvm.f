C***********************************************************************
C    Module:  getvm.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
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

      SUBROUTINE OUTVM(LU)
C...PURPOSE  Print STRIP SHEAR and BENDING QUANTITIES, ie. V, BM
C            Integrates spanload to get shear and bending moment
C     NOTE:  only works for single surface at at time (ie. V,BM on each panel)
C
      INCLUDE 'AVL.INC'
      REAL V(NSMAX), BM(NSMAX), YSTRP(NSMAX)
      CHARACTER*72 FNAMVB
C
    1 FORMAT(A)
    2 FORMAT(A,I3,A)
C
      WRITE(LU,10) TITLE(1:60),AMACH,ALFA/DTR,CLTOT,BETA/DTR,SREF,BREF
 10   FORMAT(/' Shear/q and Bending Moment/q vs Y'
     &       /'  Configuration: ',A
     &       /'  Mach  = ',F8.3,
     &       /'  alpha = ',F8.3,'    CLtot = ',F8.3,
     &       /'  beta  = ',F8.3,
     &      //'  Sref  = ',F11.5
     &       /'  Bref  = ',F11.5)
C
C---- Process the surfaces one by one, calculating shear and bending on each, 
C      with moments refered to Y=0 (centerline)
C
      DO N = 1, NSURF
        J1 = JFRST(N)
        JN = J1 + NJ(N) - 1 
C
        YMIN =  1.0E10
        YMAX = -1.0E10
        DO J = J1, JN
          IV = IJFRST(J)
          YMIN = MIN(YMIN,RV1(2,IV),RV2(2,IV))
          YMAX = MAX(YMAX,RV1(2,IV),RV2(2,IV))
        ENDDO
C
C------ Integrate spanload from first strip to last strip defined for 
C        this surface to get shear and bending moment
        CNCLST = 0.0
        BMLST  = 0.0
        WLST   = 0.0
        VLST   = 0.0
C
        JF = J1
        JL = JN
        JINC = 1
C
C------ Integrate from first to last strip in surface
        DO J = JL, JF, -JINC
          JJ = JINC*(J - JF + JINC)
C
          DY = 0.5*(WSTRIP(J)+WLST)
          YSTRP(JJ) = RLE(2,J)
          V(JJ)     = VLST  + 0.5*(CNC(J)+CNCLST) * DY 
          BM(JJ)    = BMLST + 0.5*(V(JJ)+VLST)    * DY
C
          VLST   = V(JJ) 
          BMLST  = BM(JJ)
          CNCLST = CNC(J)
          WLST   = WSTRIP(J)
        ENDDO
C
C------ Inboard edge Y,Vz,Mx
        VROOT  = VLST  +      CNCLST      * 0.5*DY 
        BMROOT = BMLST + 0.5*(VROOT+VLST) * 0.5*DY
        VTIP   = 0.0
        BMTIP  = 0.0
        IF(IMAGS(N).GE.0) THEN
         YROOT = RLE1(2,J1)
         YTIP  = RLE2(2,JN)
        ELSE
         YROOT = RLE2(2,J1)
         YTIP  = RLE1(2,JN)
        ENDIF
C
        DIR = 1.0
        IF(YMIN+YMAX.LT.0.0) DIR = -1.0
C
        WRITE(LU,*) '  '
        WRITE(LU,2) ' Surface: ',N,'  ',STITLE(N)
        WRITE(LU,*) '    2Ymin/Bref = ',2.0*YMIN/BREF
        WRITE(LU,*) '    2Ymax/Bref = ',2.0*YMAX/BREF
        WRITE(LU,*) '  2Y/Bref      Vz/(q*Sref)      Mx/(q*Bref*Sref)'
C
        WRITE(LU,4)   2.*YROOT/BREF,VROOT/SREF,DIR*BMROOT/SREF/BREF
        DO J = 1, NJ(N)
          WRITE(LU,4) 2.*YSTRP(J)/BREF,V(J)/SREF,DIR*BM(J)/SREF/BREF
        ENDDO
        WRITE(LU,4)   2.*YTIP/BREF,VTIP/SREF,DIR*BMTIP/SREF/BREF
   4    FORMAT(F10.4,G14.6,3X,G14.6)
C
C       N = 0
C       CALL GRPHIN(N,YSTRP,V)
C       CALL GRPHIN(NJ(N),YSTRP,V)
C       CALL GRPHIN(NJ(N),YSTRP,BM)
C       CALL GRPHPL
      ENDDO
C
      RETURN
      END ! OUTVM













