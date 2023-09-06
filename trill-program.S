#if __has_include("psoc-programmer-stm32/issp.h")
// this is only needed during assembly line flashing
.global trill_program_start
trill_program_start:
.incbin "../TrillRackApplication/trill_flex.bin"
.global trill_program_end
trill_program_end:
#endif
