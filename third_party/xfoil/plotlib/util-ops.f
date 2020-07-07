C--- Replacement functions for Fortran's that lack RSHIFT,LSHIFT,AND
C     Version 4.46 11/28/01

      INTEGER FUNCTION RSHIFT(I1,N)
      RSHIFT = ISHFT(I1,-N)
      RETURN
      END

      INTEGER FUNCTION LSHIFT(I1,N)
      LSHIFT = ISHFT(I1,N)
      RETURN
      END

      INTEGER FUNCTION AND(I1,I2)
      AND = IAND(I1,I2)
      RETURN
      END
