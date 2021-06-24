#lab4pre
#Okan Sen
##
## 
#################################

	.text		
	.globl __start	

__start:
	sdc2 $t0, 0($s0)
	sd $t1, $t0, label
	add $t5, $t5, $zero
	
label:
	
	li $v0,10
	syscall
	
	


	

# ------------------------------------------------------------------------




#################################
#					 	#
#     	 data segment		#
#						#
#################################

	.data




endl:			.asciiz "\n"

##
## END OF lab4pre
