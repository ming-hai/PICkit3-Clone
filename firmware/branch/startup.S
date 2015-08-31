
;---------------------------------------------------------------------
; Boot block reset vector for PK3. 
; Needed to be overloaded to stay within the ranges
;---------------------------------------------------------------------
	
.section *,code

.global __reset

;*********************************************************************
;;
;; Initialize stack, PSV, and data
;; 
;; registers used:  w0
;;
;; Inputs (defined by user or linker):
;;  __SP_init
;;  __SPLIM_init
;; 
;; Outputs:
;;  (does not return - resets the processor)
;; 
;; Calls:
;;  __psv_init
;;  __data_init
;;  _main or __main
;; 
;*********************************************************************
__reset:

    mov      #__SP_init,w15    ; initialize w15
    mov      #__SPLIM_init,w0  ; 
    mov      w0,_SPLIM         ; initialize SPLIM
    nop                        ; wait 1 cycle

    call  	_main              ; call user's main()
   
    .pword 0xDA4000            ; halt the simulator
    reset                      ; reset the processor
        
.end

