MODULE Module1
        
! ## =========================================================================== ## 
    ! MIT License
    ! Copyright (c) 2021 Roman Parak
    ! Permission is hereby granted, free of charge, to any person obtaining a copy
    ! of this software and associated documentation files (the "Software"), to deal
    ! in the Software without restriction, including without limitation the rights
    ! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    ! copies of the Software, and to permit persons to whom the Software is
    ! furnished to do so, subject to the following conditions:
    ! The above copyright notice and this permission notice shall be included in all
    ! copies or substantial portions of the Software.
    ! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    ! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    ! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    ! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    ! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    ! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    ! SOFTWARE.
    ! ## =========================================================================== ## 
    ! Author   : Roman Parak
    ! Email    : Roman.Parak@outlook.com
    ! Github   : https://github.com/rparak
    ! File Name: T_ROB1/Module1.mod
    ! ## =========================================================================== ## 
    
    ! ############### EGM Initialization Parameters ############### !
    ! Identifier for the EGM correction.
     VAR robtarget starting_position;

    ! ################################## Externally Guided motion (EGM) - Main Cycle ################################## !
    PROC main()
        starting_position := CRobT(\Tool:=Tooldata_7\wobj:=wobj0);
        ConfJ\On;
       MoveJ RelTool (starting_position, 319, 27, 71.8 \Rx:=-18.1, \Ry:=49.8, \Rz:=0), v400, z1, Tooldata_7;














  
        !MoveJ [ [0, 0, 1225], [-0.282, 0.072, 0.956, 0.021], [0, -1,-1, 1], [0, 9E9, 9E9, 9E9, 9E9, 9E9] ],v20,z1,Tooldata_7;
        !MoveJ [ [0, 0, 1000], [-0.282, 0.072, 0.956, 0.021], [0, -1,-1, 1], [0, 9E9, 9E9, 9E9, 9E9, 9E9] ],v20,z1,Tooldata_7;
        !EGM_CARTESIAN_MOVE;
       ! RAPID_MOVE;
       
    ENDPROC
    
    ! ################################## Externally Guided motion (EGM) - Cartesian Control ################################## !

    

ENDMODULE