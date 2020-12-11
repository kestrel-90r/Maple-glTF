
//
// Siv3D August 2016 v2 for Visual Studio 2019
// 
// Requirements
// - Visual Studio 2015 (v140) toolset
// - Windows 10 SDK (10.0.17763.0)
//

# include <Siv3D.hpp>

#include "3rd/mapleGltf.hpp"                   // MapleGLTF追加

using namespace s3d::Input;

enum ID_ANIME { DIG = 0, JUMP = 1, RUN = 2 ,IDLE = 3 };
Quaternion Q0001 = Quaternion(0, 0, 0, 1);      // よくある定数の略称
Float3 sca111 = Float3(1, 1, 1);
int32 WW, HH;

String GetMap(Float3 flt3,ChunkMap &chunkmap)
{
    auto idx = MaplePos2Idx(flt3, chunkmap);
    return (idx >= 0) ? chunkmap.Text.substr(idx, 1) : L"　";
}

void SetMorph(int32 ch, Entity* ent, int32 mtstate, float speed, Array<float> weight)
{
    if (ent == nullptr) return;

    auto& spd = ent->Morph[ch].Speed;
    auto& cnt = ent->Morph[ch].CntSpeed;
    auto& now = ent->Morph[ch].NowTarget;
    auto& dst = ent->Morph[ch].DstTarget;
    auto& idx = ent->Morph[ch].IndexTrans;
    auto&  wt = ent->Morph[ch].WeightTrans;

    if (now != mtstate && idx == 0)
    {
        if (ch) now = 0;
        spd = speed;
        cnt = 0;
        dst = mtstate;
        idx = 1;
        wt = (weight[0] == -1) ? WEIGHTTRANS : weight;
        wt.insert(wt.begin(), 0);
    }
}

void CtrlMorph(Entity* ent)
{
    if (ent == nullptr) return;

    for (auto ii = 0; ii < NUMMORPH; ii++)
    {
        auto& spd = ent->Morph[ii].Speed;
        auto& cnt = ent->Morph[ii].CntSpeed;
        auto& now = ent->Morph[ii].NowTarget;
        auto& dst = ent->Morph[ii].DstTarget;
        auto& idx = ent->Morph[ii].IndexTrans;
        auto&  wt = ent->Morph[ii].WeightTrans;

        if (now == -1) continue;

        cnt += spd;                     //モーフ変化量を加算
        if (cnt >= 1.0) cnt -= 1.0;
        else continue;

        if (now > -1 && idx)
        {
            idx++;
            if (wt[idx] < 0)
            {
                idx = 0;
                if (ii == 0) now = dst;  //プライマリ(表情)の場合は遷移する
                else         now = -1;   //セカンダリ(瞬き/口パク)の場合は遷移しない
            }
        }
    }
}

void CtrlKeyboard(Entity* ent, ChunkMap& map)
{
    if (ent == nullptr) return;

    const int16  DST[] = { -1, 0, 180, -1,270, 315, 225, -1,  90,  45, 135,  -1,-1,-1,-1,-1 };//押下キーに対応する方向テーブル
    static float rad = 0;
    auto& dir = ent->Rotate[0].y;       //現在の方向

    // キー入力
    uint32 key_m = 8 * KeyNum1.pressed + 4 * KeyNum3.pressed + 2 * KeyNum5.pressed + KeyNum2.pressed;
    uint32 key_a = 2 * KeyN.clicked + KeyM.clicked;

    auto& anime_s = ent->Maple->ArgI;
    // ArgI(アニメ選択)が掘るでも跳ぶでもない
    if (anime_s == ID_ANIME::IDLE || anime_s == ID_ANIME::RUN)
    {
        if (key_m && DST[key_m] != -1)  //走る
        {
            float vel = 15;              //回転の刻みは45度の公約数
            int16 dfp = 180 - abs(abs(int16(dir) + vel - DST[key_m]) - 180);
            int16 dfm = 180 - abs(abs(int16(dir) - vel - DST[key_m]) - 180);

            if (dfp < dfm) vel = +vel;
            else           vel = -vel;

            if (dir == DST[key_m]) vel = 0;
            else dir += vel;

            if (dir > 360)    dir -= 360;
            else if (dir < 0) dir += 360;

            ent->qRotate[0] *= Q0001.Yaw(Radians(vel));

//            Float3 tra = ent->qRotate[0] * Vec3::Backward * 0.1;              //旋回ターン
            Float3 tra = Q0001.Yaw(Radians(DST[key_m])) * Vec3::Backward * 0.1; //でも、直接ターンにしないと操作性悪すぎ

            //当たり判定
            auto wall_u = GetMap(ent->Trans + Float3(0, 1, 0) + tra * 10, map); // 上半身
            auto wall_l = GetMap(ent->Trans + Float3(0, 0, 0) + tra * 10, map); // 下半身 移動成分ｘ10でちょい先読み

            if (wall_u == L"　" && wall_l == L"　") ent->Trans = ent->Trans + tra;

            auto under = GetMap( ent->Trans + Float3(0, -1, 0), map) ;
            if (under == L"　") ent->Trans += Float3(0, -0.5, 0);
            else ent->Trans.y = Floor(ent->Trans.y);                            //着地時の高さを再計算

            anime_s = ID_ANIME::RUN;
        }

        if (DST[key_m] == -1 && key_a == 0) anime_s = ID_ANIME::IDLE;   //立ち
        if (KeyB.clicked) anime_s = ID_ANIME::DIG;                      //掘る
        if (KeyM.clicked) anime_s = ID_ANIME::JUMP;                     //ジャンプ
        
        if      (Key0.clicked) SetMorph(0, ent,  0, 1.0, WEIGHTTRANS);  //瞬き
        else if (Key1.clicked) SetMorph(1, ent,  1, 1.0, WEIGHTBLINK);
        else if (Key2.clicked) SetMorph(1, ent,  2, 1.0, WEIGHTBLINK);
        else if (Key3.clicked) SetMorph(2, ent,  3, 1.0, WEIGHTBLINK);
        else if (Key4.clicked) SetMorph(2, ent, 14, 1.0, WEIGHTBLINK);

        else if (Key5.clicked) SetMorph(0, ent, 10, 1.0, WEIGHTTRANS);  //表情
        else if (Key6.clicked) SetMorph(0, ent, 11, 1.0, WEIGHTTRANS);
        else if (Key7.clicked) SetMorph(0, ent, 12, 1.0, WEIGHTTRANS);
        else if (Key8.clicked) SetMorph(0, ent,  8, 1.0, WEIGHTTRANS);
        else if (Key9.clicked) SetMorph(0, ent, 14, 1.0, WEIGHTTRANS);

    }

    if (anime_s == ID_ANIME::JUMP)   //跳んでからも制御可能(マリオジャンプ)
    {
        if (key_m && DST[key_m] != -1)                     //ジャンプXZ移動
        {
            float vel = 5;         //回転の刻みは45度の公約数
            int16 dfp = 180 - abs(abs(int16(dir) + vel - DST[key_m]) - 180);
            int16 dfm = 180 - abs(abs(int16(dir) - vel - DST[key_m]) - 180);

            if (dfp < dfm) vel = +vel;
            else           vel = -vel;

            if (dir == DST[key_m]) vel = 0;
            else dir += vel;

            if (dir > 360)    dir -= 360;
            else if (dir < 0) dir += 360;

            ent->qRotate[0] *= Q0001.Yaw(Radians(vel));

            Float3 tra = Q0001.Yaw(Radians(DST[key_m])) * Vec3::Backward * 0.1; // 直接
            auto wall = GetMap(ent->Trans + tra * 10, map);                     // 移動成分ｘ10でちょい先読み

            if (wall == L"　") ent->Trans = ent->Trans + tra;
        }
    }
}


void CtrlAction(MapleMap& maple, Entity* ent, ChunkMap& chunkmap, ChunkMap& chunkatr)
{
    if (ent == nullptr) return;

    auto& anime_s = ent->Maple->ArgI;
    auto& currentframe = ent->Maple->AniModel.precAnimes[anime_s].currentframe;
    auto& animespeed = ent->Maple->ArgF;
    auto& jump = ent->CtrlF;
    auto& starty = ent->CtrlF[0];
    const Array<float> G = { 27.40f, 26.42f, 25.44f, 24.46f, 23.48f, 22.50f, 21.52f, 20.54f, 19.56f, 18.58f,
                             17.60f, 16.62f, 15.64f, 14.66f, 13.68f, 12.70f, 11.72f, 10.74f,  9.76f,  8.78f,
                              7.80f,  6.82f,  5.84f,  4.86f,  3.88f,  2.90f,  1.92f,  0.94f, -0.04f, -1.02f,
                             -2.00f, -2.98f, -3.96f, -4.94f, -5.92f, -6.90f, -7.88f, -8.86f, -9.84f,-10.82f,
                            -11.80f,-12.78f,-13.76f,-14.74f,-15.72f,-16.70f,-17.68f,-18.66f,-19.64f,-20.62f,
                            -21.60f,-22.58f,-23.56f,-24.54f,-25.52f,-26.50f,-25.20f,-0.126f,0,0
    };

//初期化
    if (ent->CtrlI[0] != anime_s)     // [0]:アニメステート履歴で切り替わった時
    {
        if      (anime_s == ID_ANIME::DIG) animespeed = 4; // 掘る 倍速設定
        else if (anime_s == ID_ANIME::RUN)  animespeed = 4;// 走る 倍速設定
        else if (anime_s == ID_ANIME::IDLE) animespeed = 2;// 立つ
        else if (anime_s == ID_ANIME::JUMP)                // 跳ぶ
        {
            starty = ent->Trans.y;
            jump.insert(jump.end(), G.begin(), G.end());
            ent->CtrlI.emplace_back(1);
            animespeed = 1;                             // 等速設定
        }

        currentframe = 1;           // フレームカウンタを1に設定してアニメスタート(ループして0で終了)
        ent->CtrlI[0] = anime_s;
    }
    else
    {
//各アニメ制御
        if (anime_s == ID_ANIME::IDLE)                 // 立ち行動の表現
        {
            static int32 cnt = 0;
            static float sign = +1;
            static float VEL[] = { 0.0, 0.1, 0.2, 0.4, 0.4, 0.4, 0.6, 0.6, 0.6, 0.6,
                                   0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6,
                                   0.6, 0.6, 0.4, 0.4, 0.4, 0.2, 0.1, -1 };
            if (VEL[cnt] == -1) cnt = 0;
            else
            {
                if (cnt == 0 && Random(0.0, 100.0) < 1.0)
                {
                    cnt = 1;
                    if (Random(0, 10) < 5) sign *= -1;
                }
            }
            animespeed = VEL[cnt] * sign;
            if (cnt) cnt++;

            auto under = GetMap(ent->Trans + Float3(0, -1, 0), chunkmap);

            if (under == L"　") ent->Trans += Float3(0, -0.5, 0);   //落下
            else                ent->Trans.y = Floor(ent->Trans.y); //着地時の高さを再計算
        }

        if (anime_s == ID_ANIME::JUMP)         // 跳び行動の表現
        {
            if (currentframe == 0 || ent->CtrlI[1] >= G.size())
            {
                anime_s = ID_ANIME::RUN;      // 終わったら2:走るに変更
                animespeed = 4; // 走る 倍速設定
                ent->Trans.y = starty;
                ent->CtrlF.resize(1);
                ent->CtrlI.resize(1);
                return;
            }

            auto tra = // ent->qRotate[0] * Vec3::Backward * 0.08 +  // xz平面の移動成分
                       Float3(0, jump[ent->CtrlI[1]++] / 190, 0);    // y軸の移動成分

            auto under = GetMap(ent->Trans + Float3(0, -1, 0) + tra, chunkmap);

            if (tra.y < 0 && under != L"　")
            {
                anime_s = ID_ANIME::RUN;  // 終わったら2:走るに変更
                animespeed = 4;           // 走る 倍速設定
                ent->CtrlF.resize(1);
                ent->CtrlI.resize(1);
                return;
            }
            else
            {
                auto wall = GetMap(ent->Trans + tra, chunkmap);
                if (wall == L"　") ent->Trans += tra;
                else
                {
                    anime_s = ID_ANIME::RUN;  // 終わったら2:走るに変更
                    animespeed = 4;         // 走る 倍速設定
                    ent->CtrlF.resize(1);
                    ent->CtrlI.resize(1);
                    return;
                }
            }
        }

        if (anime_s == ID_ANIME::DIG)         // 掘り行動の表現
        {
            if (currentframe == 0) anime_s = ID_ANIME::IDLE; // 終わったら3:立つに変更
        }

        //床の重力ベクトル対応(床が動いていたら一緒に動かす)
        auto map_under = GetMap(ent->Trans + Float3(0, -1, 0), chunkmap);
        if (map_under != L"　")  //足元に何かある
        {
            auto atr_under = GetMap(ent->Trans + Float3(0, -1, 0), chunkatr);
            for (auto& atr : maple.Attribs)
            {
                if (atr.Name == atr_under)                               //取ってきた属性を選択
                {
                    ent->Trans += atr.Trans;                             //属性の重力ベクトルを加算
                    break;
                }
            }
        }
    }
}

void Main()
{
    String maplepath = FileSystem::CurrentPath()+L"Example/MapleGLTF/";     // リソースのパス

    // マップ定義ファイルから座標リストとマップを生成
    CSVReader csv1(maplepath + L"map.csv");
    MapleMap maple1;                                // 物体の座標リスト
    ChunkMap map1, atr1;                            // 物体と属性のマップ
    LoadMaple(maple1, map1, atr1, csv1, maplepath);

    Window::Resize(1366, 768);
    WW = Window::Width();  HH = Window::Height();

    const Font FontT(15, L"TOD4UI");    // 英数フォント
    const Font FontM(7);                // 日本語フォント

    // カメラ設置
    Camera camera = Graphics3D::GetCamera();
    camera.pos = Float3(0.32, 4, 20);    camera.lookat = Float3(16, 1, -16);
    Graphics3D::SetCamera(camera);

    // 背景色
    Graphics::SetBackground(Palette::Midnightblue);
    Graphics3D::SetAmbientLight(ColorF(0.7, 0.7, 0.7));

    //専用制御(モーフ/可変アニメ)のEntityを取得
    Entity* ent_s = nullptr;        
    Entity* ent_d = nullptr;        
    Entity* ent_f = nullptr;        
    for (auto& ent : maple1.Entities)
    {
        if (ent.Name == L"Ｓ") //SIV3DKUNの初期化
        {
            ent_s = &ent;
            ent_s->Maple->ArgI = ID_ANIME::IDLE; //アニメ種類3：待ち
        }
        if (ent.Name == L"Ｄ") //情報ブロックの初期化
        {
            ent_d = &ent;
            ent_d->Count = 100;
        }
        if (ent.Name == L"文") //文字列の初期化
        {
            ent_f = &ent;
        }
    }

    //ストップウォッチ開始
//    Stopwatch stopwatch;
//    stopwatch.start();              

    while (System::Update())
    {
        auto begin = std::chrono::high_resolution_clock::now();

        Graphics3D::FreeCamera();
        camera = Graphics3D::GetCamera();

        CtrlKeyboard(ent_s, map1);                  // キー入力制御 [1]：←[2]：↓[3]：→[5]：↑[M]：ジャンプ[B]:掘る
        CtrlMorph(ent_s);                           // モーフィング制御
        CtrlAction(maple1, ent_s, map1, atr1);      // 行動制御
        RenderMaple(maple1, map1, atr1);            // 描画

        //情報ブロック表示
        if (ent_d)
        {
            ent_d->Trans = ent_s->Trans;
            ent_d->Trans.y = ent_s->Trans.y + 1;
            ent_d->Maple->Mode = String(L"円:MapPosition X=(X) Y=(Y) Z=(Z)")
                .replace(L"(X)", Format(ent_s->Trans.x))
                .replace(L"(Y)", Format(ent_s->Trans.y))
                .replace(L"(Z)", Format(ent_s->Trans.z));
        }

        //文字列表示
        if (ent_f)
        {
            static float vel = 0.1;
            ent_f->Count += vel;
            if ((vel > 0 && ent_f->Count > ent_f->Maple->Mode.length-2) || 
                (vel < 0 && ent_f->Count < 0) ) vel *= -1;
        }

        //物体マップ表示
        auto &dd = map1.MapSize.z;
        auto &ww = map1.MapSize.x;
        for (int32 yy = 0; yy < dd ; yy++)           
        {
            for (int32 xx = 0; xx < ww; xx++)
            {
                int32 idx = MaplePos2Idx(xx, 1, (dd-1) - yy, 32, 32, 32);
                auto cc = map1.Text.substr(idx, 1);
                FontM(cc).draw(xx * 8, yy * 8);
            }
        }

        //フレームレート表示
        Profiler::FPS();

        FontT(String(L"Render:(FPS)/Fps")
            .replace(L"(FPS)", Format(Profiler::FPS())))
            .draw(WW - 300, HH - 80);

    }
}
