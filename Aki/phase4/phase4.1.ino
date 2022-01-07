//phase4_1
//CanSatLogData.plintln();

/*
 * SC-2と比較してわからなかったところ
 * いくつかの関数 と変数
 * ex) M_2_pi,measuring(),Calculatedis(),CalculateAngel()
 * 
 * while文を結構多用していたのでなくしました．うまく変えれてるか確認できない．(センサー系はMPUとGY-271しか持ってないため)
 */

void setup() {

  //you need to set up variables at first
  
  //those are variables which is used in GPS
  float GPSlat,GPSlng; //GPS緯度，経度
  double dis_goal; //ゴールとの距離を表す関数
  double now_dis,now_rad,next_dis,next_rad,delta_rad;

  //those are variables which is used in 超音波
  double s_dis;


//it will be if(phase state !=4)
Serial.write("phase4: transition completed! \n");

}




void loop() {
  
  //ループに入った時の位置
  now_dis = Calculatedis();
  now_rad = CalculateAngel();


  //距離が遠いときはGPS利用
  if(now_dis > 3){
    forward(1000);   //前に進む関数は別にあるのかな？そもそも前に進んでいいのか？
    next_rad = CalcurateAngel();//進んだ後角度確認
    delta_rad = next_rad - now_rad;

    //ゴールより右に向いてる時
    if(delta_rad > 0){   //左回転
      levoversion(delta_rad * M_2_pi * 1000);  //M_2_piの関数見つからず
    }   
    //ゴールより左に向いてる時
    else if(delta_rad < 0){   //右回転
      dextroversion(abs(delta_rad) * M_2_pi * 1000);  //角度は絶対値に変えて計算
    }
  }//ここまでGPS


  //距離が近くなったら超音波使用
  else if(now_dis <= 3){
    s_dis = measuring();    //おそらく超音波から得た値がmeasuring()　で単位mかな？

    //ここから下ちょっと変えたい気もする
    //距離が10mより遠いとき，機体の向きが違うと考えられるため回転 (10mは大きすぎ?)
    if(s_dis > 10){
      dextroversion(1000); //右回転
      stopping();  //stopping いる？
    }
    //10m以内だけどまだ離れてる時
    else if(Searchdis <= 10 && Searchdis != 0){
      forward(3000);  //前へ
      stopping();  //stopping いる？
    }
    
  }//ここまで超音波
  

}
