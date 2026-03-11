ModeScreenTable:	.dw	Mode0Screen
					.dw	Mode1Screen
					.dw	Mode2Screen
					.dw	Mode3Screen

Mode0Screen:	.db F,0,XY,2,0,"выход  ",\
					F,2,XY,104,0,Range,\
					F,1,XY,0,3,MainV,\
					Line,2,0,127,$08,\
					EOS

Mode1Screen:	.db F,0,XY,0,0,"разгон ",\
					F,2,XY,78,0,TargetV,\
					Line,2,0,127,$08,\
					F,1,XY,0,3,OutputV,\
					EOS

Mode2Screen:	.db F,0,XY,0,0,"выкл   ",\
					F,2,XY,78,0,TargetV,\
					Line,2,0,127,$08,\
					F,1,XY,0,3,OutputV,\
					EOS

Mode3Screen:	.db F,0,XY,0,1,HS," выключено  ",\
						XY,0,3,HS," аварийным  ",\
						XY,0,5,"  сигналом  ",EOS


