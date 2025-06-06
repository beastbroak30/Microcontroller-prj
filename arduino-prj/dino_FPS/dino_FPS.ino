#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include "sprite.c"

const int TICKS_PER_SECOND = 50;
const int SKIP_TICKS = 1000 / TICKS_PER_SECOND;
const int MAX_FRAMESKIP = 40;

const int trunk_w = 29;
const int trunk_l = 25;
const int leg_w = 29;
const int leg_l = 6;
int dino_y = 97;
int y;
int y_old;
int tree_pos = 80;
int dino_x = 35;
int type_old;

int type = 2;
int color;
float cloud_pos;
int cloud_height;
int cloud_type;
float cloud_pos1;
int cloud_height1;
int cloud_type1;

double diff;
float u;
bool button_press = false;
bool game_running = false;
bool main_menu=true;
unsigned long next_game_tick;
unsigned long game_F;
unsigned long t1;
unsigned long t2;
unsigned long t3;
unsigned long score;

int loops;
float interpolation;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void button() {
  if (game_running == true) {
    t2 = millis();
    u = 90.0;
    button_press = true;
    tone(7, 523, 30);
    //Serial.println("Pressed");
  } else {
    main_menu = false;
    game_running = true;
    tft.fillScreen(tft.color565( 0xff, 0xff, 0xff));
    score = millis();

  }

}

void game_over() {
  game_running = false;
  tft.setCursor(25, 50);
  tft.invertDisplay(true);
  tft.setTextColor(ST7735_RED);
  tft.invertDisplay(false);
  tft.setTextSize(2);
  tft.invertDisplay(true);
  tft.print("Game Over!");
  tft.invertDisplay(false);
  tone(7, 523, 60);
  tft.invertDisplay(true);
  tone(7, 230, 30);
  tft.invertDisplay(false);
  tone(7, 523, 60);
  tft.invertDisplay(true);
  tone(7, 230, 30);
  tft.invertDisplay(false);

  //add some music + Game over classic display text
}

void update_game() {
  t1 = millis();
  if (button_press) {
    diff = (abs(t1 - t2)) / 1000.00;
    y = u * diff - 35 * diff * diff;
    //Serial.println(y, 8);
    if (y <= 0 && diff > 1) {             //detect touch down
      u = 0;
      y = 0;
      t2 = 0;
      button_press = false;
      //Serial.println("Down");
    }
  }
  if (t3 < t1) {
    t3 = t1 + 3000;
    type = random(1, 3);

    if (type_old != type) {
      if (type_old == 1) {
        tft.fillRoundRect(tree_pos, 99, 4, 29, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 10, 113, 20, 4, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 10, 113 - 8, 4, 10, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos + 8, 113 - 10, 4, 12, 1.5, tft.color565( 0xff, 0xff, 0xff));
      } else if (type_old == 2 ) {
        tft.fillRoundRect(tree_pos, 108, 4, 20, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 8, 115, 18, 4, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 9, 115 - 8, 4, 10, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos + 8, 115 - 8, 4, 10, 1.5, tft.color565( 0xff, 0xff, 0xff));

        tft.fillRoundRect(tree_pos + 25, 108, 4, 20, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 8 + 25, 115, 18, 4, 1.5, tft.color565( 0xff, 0xff, 0xff));
        tft.fillRoundRect(tree_pos - 9 + 25, 115 - 8, 4, 10, 1.5, tft.color565( 0xff, 0xff, 0xff));
        //tft.fillRoundRect(tree_pos+8+25,115-8,4,10,1.5,tft.Color565( 0xff, 0xff, 0xff));

      }
      type_old = type;
      tree_pos = 160;

    }
  }
  if ( type == 1 && ((dino_y - y + trunk_l + leg_l) > (128 - 29)) && (dino_x > (tree_pos - 10)) && (dino_x < (tree_pos + 8)) ) {
    game_over();
  } else if ( type == 2 && ((dino_y - y + trunk_l + leg_l) > (128 - 20)) && (dino_x > (tree_pos - 9)) && (dino_x < (tree_pos + 35)) ) {
    game_over();
  }
}

void display_game() {
  //check if the dino pose changed, if yes rewrite the old one with background and write the new one else only do leg animation
  //use game_F to switch between leg frames (odd/even)
  if (y_old != y) {
    drawBitmap(dino_x, dino_y - y, body, trunk_w, trunk_l, 0x0000);
    drawBitmap(dino_x, dino_y - y + trunk_l, leg1, leg_w, leg_l,  0x0000);
    tft.fillRect(dino_x, dino_y - y - 5, trunk_w + 5, trunk_l + 5, tft.color565( 0xff, 0xff, 0xff)); //clearing extra bits
    //drawBitmap(dino_x, dino_y - y, body, 40, 35, tft.Color565( 0xff, 0xff, 0xff));
    tft.fillRect(dino_x, dino_y - y + trunk_l - 5, leg_w + 5, leg_l + 10, tft.color565( 0xff, 0xff, 0xff));
    //drawBitmap(dino_x, dino_y - y+ 35, leg1, 40, 8, tft.Color565( 0xff, 0xff, 0xff));
    y_old = y;
    delay(2);
  } else
  {
    drawBitmap(dino_x, dino_y - y, body, trunk_w, trunk_l,  0x0000);
    if (game_F % 4 == 0) {
      //drawBitmap(dino_x, dino_y - y+ 35, leg2, 40, 8, tft.Color565( 0xff, 0xff, 0xff));
      tft.fillRect(dino_x, dino_y - y + trunk_l, leg_w, leg_l, tft.color565( 0xff, 0xff, 0xff));
      drawBitmap(dino_x, dino_y - y + trunk_l, leg1, leg_w, leg_l,  0x0000);
      //delay(250);

    }
    else {
      tft.fillRect(dino_x, dino_y - y + trunk_l, leg_w, leg_l, tft.color565( 0xff, 0xff, 0xff));
      drawBitmap(dino_x, dino_y - y + trunk_l, leg2, leg_w, leg_l,  0x0000);
      //delay(250);

    }
    game_F++;
  }
  if (type == 1 ) {
    tft.fillRoundRect(tree_pos, 99, 4, 29, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 10, 113, 20, 4, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 10, 113 - 8, 4, 10, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos + 8, 113 - 10, 4, 12, 1.5, tft.color565(  0xff,  0xff,  0xff));
  } else if (type == 2 ) {
    tft.fillRoundRect(tree_pos, 108, 4, 20, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 8, 115, 18, 4, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 9, 115 - 8, 4, 10, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos + 8, 115 - 8, 4, 10, 1.5, tft.color565(  0xff,  0xff,  0xff));

    tft.fillRoundRect(tree_pos + 25, 108, 4, 20, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 8 + 25, 115, 18, 4, 1.5, tft.color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(tree_pos - 9 + 25, 115 - 8, 4, 10, 1.5, tft.color565(  0xff,  0xff,  0xff));
    //tft.fillRoundRect(tree_pos+8+25,115-8,4,10,1.5,tft.Color565(  0xff,  0xff,  0xff));

  }

  if (tree_pos > 0) {
    tree_pos = tree_pos - 5;
    color = ST7735_BLACK;
  } else {
    color = tft.color565(  0xff,  0xff,  0xff); // to avoid stray pixels due to border effects
  }



  if (type == 1) {
    tft.fillRoundRect(tree_pos, 99, 4, 29, 1.5, color);
    tft.fillRoundRect(tree_pos - 10, 113, 20, 4, 1.5, color);
    tft.fillRoundRect(tree_pos - 10, 113 - 8, 4, 10, 1.5, color);
    tft.fillRoundRect(tree_pos + 8, 113 - 10, 4, 12, 1.5, color);
  } else if (type == 2 ) {
    tft.fillRoundRect(tree_pos, 108, 4, 20, 1.5, color);
    tft.fillRoundRect(tree_pos - 8, 115, 18, 4, 1.5, color);
    tft.fillRoundRect(tree_pos - 9, 115 - 8, 4, 10, 1.5, color);
    tft.fillRoundRect(tree_pos + 8, 115 - 8, 4, 10, 1.5, color);

    tft.fillRoundRect(tree_pos + 25, 108, 4, 20, 1.5, color);
    tft.fillRoundRect(tree_pos - 8 + 25, 115, 18, 4, 1.5, color);
    tft.fillRoundRect(tree_pos - 9 + 25, 115 - 8, 4, 10, 1.5, color);
    //tft.fillRoundRect(tree_pos+8+25,115-8,4,10,1.5,ST7735_GREEN);

  }
  if (cloud_type == 1) {
    //drawBitmap(cloud_pos, cloud_height, cloud_big, 18, 8, tft.Color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(cloud_pos, cloud_height, 18, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  } else if (cloud_type == 2) {
    //drawBitmap(cloud_pos, cloud_height, cloud_small, 11, 8, tft.Color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(cloud_pos, cloud_height, 11, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  }
  if (cloud_pos > 0) {
    cloud_pos = cloud_pos - 0.5;
  }
  else {
    if (cloud_type == 1) {
    tft.fillRoundRect(cloud_pos, cloud_height, 18, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  } else if (cloud_type == 2) {
    tft.fillRoundRect(cloud_pos, cloud_height, 11, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  }
    cloud_pos = random(30,161);
    cloud_type = random(1, 3);
    cloud_height = random(20, 30);
  }
  if (cloud_type == 1) {
    tft.fillRoundRect(cloud_pos, cloud_height, 18, 8, 1.5, tft.color565(  0x80,  0x80,  0x80));
    //drawBitmap(cloud_pos, cloud_height, cloud_big, 18, 8, tft.Color565(  0x00,  0x00,  0x00));
  } else if (cloud_type == 2) {
    tft.fillRoundRect(cloud_pos, cloud_height, 11, 8, 1.5, tft.color565(  0x80,  0x80,  0x80));
    //drawBitmap(cloud_pos, cloud_height, cloud_small, 11, 8, tft.Color565(  0x00,  0x00,  0x00));
  }

  

  if (cloud_type1 == 1) {
    //drawBitmap(cloud_pos, cloud_height, cloud_big, 18, 8, tft.Color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(cloud_pos1, cloud_height1, 18, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  } else if (cloud_type1 == 2) {
    //drawBitmap(cloud_pos, cloud_height, cloud_small, 11, 8, tft.Color565(  0xff,  0xff,  0xff));
    tft.fillRoundRect(cloud_pos1, cloud_height1, 11, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  }
  if (cloud_pos1 > 0) {
    cloud_pos1 = cloud_pos1 - 0.5;
  }
  else {
    if (cloud_type1 == 1) {
    tft.fillRoundRect(cloud_pos1, cloud_height1, 18, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  } else if (cloud_type1 == 2) {
    tft.fillRoundRect(cloud_pos1, cloud_height1, 11, 8, 1.5, tft.color565(  0xff,  0xff,  0xff));
  }
    cloud_pos1 = random(30,161);
    cloud_type1 = random(1, 3);
    cloud_height1 = random(20, 40);
  }
  if (cloud_type1 == 1) {
    tft.fillRoundRect(cloud_pos1, cloud_height1, 18, 8, 1.5, tft.color565(  0x80,  0x80,  0x80));
    //drawBitmap(cloud_pos, cloud_height, cloud_big, 18, 8, tft.Color565(  0x00,  0x00,  0x00));
  } else if (cloud_type1 == 2) {
    tft.fillRoundRect(cloud_pos1, cloud_height1, 11, 8, 1.5, tft.color565(  0x80,  0x80,  0x80));
    //drawBitmap(cloud_pos, cloud_height, cloud_small, 11, 8, tft.Color565(  0x00,  0x00,  0x00));
  }


}

void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if (byte & 0x80) tft.drawPixel(x + i, y + j, color);
    }
  }
}


void setup() {
  next_game_tick = millis();
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), button, FALLING);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(tft.color565(  0xff,  0xff,  0xff));
  tft.setCursor(25, 40);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  tft.print("Dino Game");
  tft.setCursor(20,60);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(1);
  tft.print("Press ENTER to start");
  tft.setCursor(50,75);
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.print("By: akar@Beastbroak");
  

  drawBitmap(10, 97,body,trunk_w,trunk_l, 0x0000);
  //game_over();
  //drawBitmap(20, 20,cloud_big,31,16,tft.Color565(  0x00,  0x00,  0x00));
  //drawBitmap(20, 40,cloud_small,21,16,tft.Color565(  0x00,  0x00,  0x00));




}

void loop() {
  if(main_menu){
    drawBitmap(10, 97+25,leg1,leg_w,leg_l, 0x0000);
    delay(250);
    drawBitmap(10, 97+25,leg1,leg_w,leg_l, tft.color565(  0xff,  0xff,  0xff));
    drawBitmap(10, 97+25,leg2,leg_w,leg_l, 0x0000);
    delay(250);
    drawBitmap(10, 97+25,leg2,leg_w,leg_l, tft.color565(  0xff,  0xff,  0xff));
  }
  while (game_running) {
    tft.fillRect(80, 2, 80, 15, tft.color565(  0xff,  0xff,  0xff));
    tft.setCursor(80, 2);
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(2);
    tft.print((millis() - score) / 1000);
    //update_game();
    //display_game();
    loops = 0;
    while ( millis() > next_game_tick && loops < MAX_FRAMESKIP) {
      update_game();

      next_game_tick += SKIP_TICKS;
      loops++;
    }

    display_game();


  }

}
