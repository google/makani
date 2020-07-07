
C
C---- Pointers for referencing polar force coefficients
C     First 4 pointers must be main polar plot variables.
C
      PARAMETER (
     &    IAL =  1,    ! alpha
     &    ICL =  2,    ! CL
     &    ICD =  3,    ! CD
     &    ICM =  4,    ! Cm
     &    ICW =  5,    ! CDwave
     &    ICV =  6,    ! CDvisc
     &    ICP =  7,    ! CDpres
     &    IMA =  8,    ! Mach
     &    IRE =  9,    ! Re
     &    ICH = 10,    ! Hinge moment
     &    IMC = 11,    ! Minimum Cp on surface
     &    ICDH   = 12,   ! CDh  (engine thrust coeff.)
     &    ICMDOT = 13 )  ! Cm_dot
      PARAMETER (IPTOT=13)
C
C
C---------------------
C  Pointers for referencing polar airfoil-side quantities
C
      PARAMETER (
     &    JNC = 1,     ! Ncrit
     &    JTP = 2,     ! trip
     &    JTN = 3,     ! transition
     &    JTI = 4 )    ! transition index
      PARAMETER (JPTOT=4)

      CHARACTER*10 CPOLNAME(IPTOT)
      CHARACTER*5 CPOLSNAME(JPTOT)
      CHARACTER*6 CPOLFORM(IPTOT), CPOLSFORM(JPTOT)
C
      DATA CPOLNAME  /
     & 'alpha     ',
     & 'CL        ',
     & 'CD        ',
     & 'CM        ',
     & 'CDw       ',
     & 'CDv       ',
     & 'CDp       ',
     & 'Mach      ',
     & 'Re        ',
     & 'Chinge    ', 
     & 'Cpmin     ',
     & 'CDh       ',
     & 'Cmdot     '  /
      DATA CPOLFORM  /
     & 'F7.3  ',   !    alpha
     & 'F9.4  ',   !     CL  
     & 'F10.5 ',   !     CD  
     & 'F9.4  ',   !     CM  
     & 'F10.5 ',   !     CDw 
     & 'F10.5 ',   !     CDv 
     & 'F10.5 ',   !     CDp 
     & 'F8.4  ',   !     Mach
     & 'E11.3 ',   !     Re  
     & 'F10.5 ',   !    Chinge
     & 'F9.4  ',   !    Cpmin 
     & 'F11.5 ',   !     CDh
     & 'F9.5  ' /  !     Cmdot 

      DATA CPOLSNAME /
     & 'Ncrit',
     & 'Xtrip',
     & 'Xtr  ',
     & 'Itr  ' /
      DATA CPOLSFORM  /
     & 'F7.3  ',    !    Ncrit
     & 'F9.4  ',    !    Xtrip
     & 'F9.4  ',    !    Xtr
     & 'F9.4  '  /  !    Itr
