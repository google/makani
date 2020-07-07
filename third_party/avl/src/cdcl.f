C***********************************************************************
C    Module:  cdcl.f
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

      SUBROUTINE CDCL(J,CL,CD,CD_CL)
C
C--- Routine for CD as a function of CL, as defined by 6 parameters
C    defining the polar, input by the user and stored for each strip J.
C
C    The CD-CL polar is based on a simple interpolation scheme using
C    four CL regions:
C     1) negative stall region
C     2) parabolic CD(CL) region between negative stall and the drag minimum
C     2) parabolic CD(CL) region between the drag minimum and positive stall
C     4) positive stall region
C
C             CLpos,CDpos       <-  Region 4 (quadratic above CLpos)
C    CL |     X--------      
C       |    /                     
C       |   |                   <-  Region 3 (quadratic above CLcdmin)
C       |   X CLcdmin,CDmin  
C       |   |                
C       |    \                  <-  Region 2 (quadratic below CLcdmin)
C       |     X_________      
C       |     CLneg,CDneg       <-  Region 1 (quadratic below CLneg)
C       |                             
C       -------------------------
C                       CD
C
C    The parameters that control the section "polar" are:
C      CLCDSEC(1)  the CL (CLneg) at point before the negative stall drag rise
C      CLCDSEC(2)  the CD (CDneg) at CLneg
C      CLCDSEC(3)  the CL (CLcdmin) at minimum drag (middle of polar)
C      CLCDSEC(4)  the CD (CDmin) at CLcdmin
C      CLCDSEC(5)  the CL (CLpos) at point before the positive stall drag rise
C      CLCDSEC(6)  the CD (CDpos) at CLpos
C
      INCLUDE 'AVL.INC'
C
C--- CLINC and CDINC control the rate of increase of drag in the stall regions
C    CLINC=0.2 forces drag to increase by CDINC over deltaCL=0.2 after stall
C    The CDINC term is the CD increment added by deltaCL=CLINC after stall
C
      DATA  CLINC, CDINC / 0.2, 0.0500 /
C
      CD    = 0.0
      CD_CL = 0.0
      IF(J.LT.1 .OR. J.GT.NSTRIP) THEN
        WRITE(*,*) 'Error in CDCL - strip index out of bounds'
        RETURN
      ENDIF
C
C--- Unpack the CD,CL parameters for this strip
      CLMIN = CLCD(1,J)
      CDMIN = CLCD(2,J)
      CL0   = CLCD(3,J)
      CD0   = CLCD(4,J)
      CLMAX = CLCD(5,J)
      CDMAX = CLCD(6,J)
      IF(CLMAX.LE.CL0 .OR. CL0.LE.CLMIN) THEN
        WRITE(*,*) '*** CDCL input CL data out of order'
        RETURN
      ENDIF
C
C--- Some matching parameters that make the slopes smooth at the stall joins
      CDX1 = 2.0*(CDMIN-CD0)*(CLMIN-CL0)/(CLMIN-CL0)**2 
      CDX2 = 2.0*(CDMAX-CD0)*(CLMAX-CL0)/(CLMAX-CL0)**2 
      CLFAC = 1.0 / CLINC
C
C--- Four formulas are used for CD, depending on the CL  
C
      IF(CL.LT.CLMIN) THEN
C--- Negative stall drag model (slope matches lower side, quadratic drag rise)
        CD = CDMIN + CDINC*(CLFAC*(CL-CLMIN))**2
     &      + CDX1*(1.0 - (CL-CL0)/(CLMIN-CL0))
        CD_CL = CDINC*CLFAC*2.0*(CL-CLMIN)
C
C--- Quadratic matching negative stall and minimum drag point with zero slope
       ELSEIF(CL.LT.CL0) THEN
        CD = CD0 + (CDMIN-CD0)*(CL-CL0)**2/(CLMIN-CL0)**2  
        CD_CL = (CDMIN-CD0)*2.0*(CL-CL0)/(CLMIN-CL0)**2  
C
C--- Quadratic matching positive stall and minimum drag point with zero slope
       ELSEIF(CL.LT.CLMAX) THEN
        CD = CD0 + (CDMAX-CD0)*(CL-CL0)**2/(CLMAX-CL0)**2  
        CD_CL = (CDMAX-CD0)*2.0*(CL-CL0)/(CLMAX-CL0)**2  
C
       ELSE
C--- Positive stall drag model (slope matches upper side, quadratic drag rise)
        CD = CDMAX + CDINC*(CLFAC*(CL-CLMAX))**2  
     &      - CDX2*(1.0 - (CL-CL0)/(CLMAX-CL0))
        CD_CL = CDINC*CLFAC*2.0*(CL-CLMAX)
      ENDIF
C
      RETURN
      END







