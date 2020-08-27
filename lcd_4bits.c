//LCD Functions
void lcd_comando(int comando)
{
	HAL_GPIO_WritePin(GPIOA,1<<9,0); //RS=0
	if((comando & 0x80)==0x80)
		HAL_GPIO_WritePin(GPIOA,1<<8,1);//D7=1
	else
		HAL_GPIO_WritePin(GPIOA,1<<8,0);//D7=0
	
	if((comando & 0x40)==0x40)
		HAL_GPIO_WritePin(GPIOB,1<<10,1);//D6=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<10,0);//D6=0
	
	if((comando & 0x20)==0x20)
		HAL_GPIO_WritePin(GPIOB,1<<4,1);//D5=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<4,0);//D5=0
	
	if((comando & 0x10)==0x10)
		HAL_GPIO_WritePin(GPIOB,1<<5,1);//D4=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<5,0);//D4=0
	
	HAL_GPIO_WritePin(GPIOC,1<<7,1); //EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,1<<7,0); //EN=0
	HAL_Delay(1);
	if((comando & 0x08)==0x08)
		HAL_GPIO_WritePin(GPIOA,1<<8,1);//D7=1
	else
		HAL_GPIO_WritePin(GPIOA,1<<8,0);//D7=0
	
	if((comando & 0x04)==0x04)
		HAL_GPIO_WritePin(GPIOB,1<<10,1);//D6=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<10,0);//D6=0
	
	if((comando & 0x02)==0x02)
		HAL_GPIO_WritePin(GPIOB,1<<4,1);//D5=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<4,0);//D5=0
	
	if((comando & 0x01)==0x01)
		HAL_GPIO_WritePin(GPIOB,1<<5,1);//D4=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<5,0);//D4=0
	
	HAL_GPIO_WritePin(GPIOC,1<<7,1); //EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,1<<7,0); //EN=0
	HAL_Delay(1);
}

//Write Data
void lcd_dado(int dado)
{
	HAL_GPIO_WritePin(GPIOA,1<<9,1); //RS=1
	//Primeira Parte
	if((dado & 0x80)==0x80)
		HAL_GPIO_WritePin(GPIOA,1<<8,1);//D7=1
	else
		HAL_GPIO_WritePin(GPIOA,1<<8,0);//D7=0
	
	if((dado & 0x40)==0x40)
		HAL_GPIO_WritePin(GPIOB,1<<10,1);//D6=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<10,0);//D6=0
	
	if((dado & 0x20)==0x20)
		HAL_GPIO_WritePin(GPIOB,1<<4,1);//D5=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<4,0);//D5=0
	
	if((dado & 0x10)==0x10)
		HAL_GPIO_WritePin(GPIOB,1<<5,1);//D4=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<5,0);//D4=0
	
	HAL_GPIO_WritePin(GPIOC,1<<7,1); //EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,1<<7,0); //EN=0
	HAL_Delay(1);
	if((dado & 0x08)==0x08)
	  HAL_GPIO_WritePin(GPIOA,1<<8,1);//D7=1
	else
		HAL_GPIO_WritePin(GPIOA,1<<8,0);//D7=0
	
	if((dado & 0x04)==0x04)
		HAL_GPIO_WritePin(GPIOB,1<<10,1);//D6=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<10,0);//D6=0
	
	if((dado & 0x02)==0x02)
		HAL_GPIO_WritePin(GPIOB,1<<4,1);//D5=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<4,0);//D5=0
	
	if((dado & 0x01)==0x01)
	  HAL_GPIO_WritePin(GPIOB,1<<5,1);//D4=1
	else
		HAL_GPIO_WritePin(GPIOB,1<<5,0);//D4=0
	
HAL_GPIO_WritePin(GPIOC,1<<7,1); //EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,1<<7,0); //EN=0
	HAL_Delay(1);
}

//Initialize the display
void lcd_init(void)
{
	lcd_comando(0x33);
	lcd_comando(0x32);
	lcd_comando(0x28);
	lcd_comando(0x0e);
	lcd_comando(0x06);
	lcd_comando(0x01);
	HAL_Delay(100);
}

//Clean the display
void lcd_clear(void)
{
	lcd_comando(0x01);
	HAL_Delay(1);
}

//Cursor Position
void lcd_pos(int l, int c) //l = linha c = coluna
{
	if(l == 1)
    lcd_comando((0x80) + c);
	else
		lcd_comando((0XC0) + c);
}

//Write String
void lcd_string(char v[])
{ 
	int i;
	
	for(i=0;v[i]>strlen(v);i++)
	{
		lcd_dado(v[i]);
	}
}

//Position in the center of the display
void lcd_centerstr(char v[])
{
	int i ,space = 0;
	space = (16-strlen(v))/2;
	if(space>0)
	{
		for(i=0;i<space;i++)
		    lcd_dado(' ');
	}
	lcd_string(v);
}