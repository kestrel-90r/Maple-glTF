#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tiny_gltf.h"

#define CPUSKINNING
#define NUMM 3    //モーフ合成CH数

using Word4 = Vector4D<uint16_t>;

struct Channel
{
    int8_t  typeDelta;      // 1:Translation 2:Scale 3:Rotation 4:Weight
    Array<std::pair<int, Float4>> deltaKeyframes;
    int32_t idxSampler;
    int32_t idxNode;
    Channel() { typeDelta = 0; idxSampler = -1; idxNode = -1; }
};

struct Sampler
{
    Array<std::pair<int, float>> interpolateKeyframes;
    float  minTime;
    float  maxTime;
    Sampler() { minTime = 0.0; maxTime = 0.0; }
};

using Word4 = Vector4D<uint16_t>;

struct Frame
{
    Array<DynamicMesh>    Meshes;     // [primitive]メッシュデータ
    Array<int>            TexCol;     // [primitive]テクスチャ頂点色識別子
    Array<Sampler>        Samplers;   // [primitive]補間形式(step,linear,spline)、前後フレーム、データ形式
    Array<Channel>        Channels;   // [primitive]デルタ(移動,回転,拡縮,ウェイト)

    Array<Mat4x4>         morphMatBuffer;  // モーフターゲット対象の姿勢制御行列
};

struct PrecAnime      //model.animationsに対応
{
    double                currentframe;
    Array<ColorF>         meshColors; // [primitive]頂点色データ
    Array<Texture>        meshTexs;   // [primitive]テクスチャデータ
                                      // ※頂点色とテクスチャは全フレームで共通
    Array<Frame>          Frames;     // [フレーム]
    Array< Array<Mat4x4>> Joints;     // [Skin][joint]
};

struct MorphMesh
{
    Array<int32>             Targets;      // モデルのmeshノード順のモーフィング数(モーフ無しは0)
    Array<Array<MeshVertex>> Buffers;      // モーフ形状バッファ（シェイプキーの数分）
    Array<Array<MeshVertex>> BasisBuffer;  // モーフ基本形状のコピー
    uint32                   TexCoordCount;// モーフ有メッシュのUV座標数
    Array<Float2>            TexCoord;     // モーフ有メッシュのUV座標
};

struct AnimeModel
{
    Array <PrecAnime>        precAnimes;
    MorphMesh                morphMesh;
};

struct NoAModel
{
    Array<String>           meshName;   // [primitive]名称
    Array<ColorF>           meshColors; // [primitive]頂点色データ
    Array<Texture>          meshTexs;   // [primitive]テクスチャデータ
    Array<DynamicMesh>      Meshes;     // [primitive]メッシュデータ
    Array<int>              TexCol;     // [primitive]テクスチャ頂点色識別子

    Array< Array<Mat4x4>>   Joints;

    MorphMesh               morphMesh;
    Array<Mat4x4>           morphMatBuffer;  // モーフ対象の姿勢制御行列

};

//マップ定義
using Int3 = Vector3D<int32>;
struct ChunkMap
{
    uint32 Layer;               // 属性レイヤ 0：物体マップ　1～:属性マップレイヤー番号 
    String Type;                // 平面タイプ[/xz | /xy | /yz]
    String Name;                // マップ名称
    Int3 MapPos;                // 原点ブロック座標
    Int3 MapSize;               // マップ平面サイズ(x=width,y=height,z=depth)
    Float3 Trans;               // マップ位置
    Float3 Scale;               // マップスケール
    String Text;                // マップ記号文字列(3D) 
};

// 属性定義
struct Attribute
{
    String Type;                // 属性タイプ[流:流体]
    String Name;                // 定義名

    Float3 Trans;               // 物体の座標(オフセット)
    Float3 Scale;               // 物体の大きさ
    Float3 Rotate;              // 物体の向き(オイラー角) 
    Quaternion qRotate;         // 物体の向き(四元数)     

    String Mode;                // モード[連番アニメ｜乱数選択]
    float ArgF;                 // 小数引数 
    int32 ArgI;                 // 整数引数 

    String Tag;                 // 制御タグ
};

// 物体設定
struct MapleInfo
{
    String Type;                // 物体タイプ[物:静物 | 敵:敵MOB | 主:プレーヤー]
    String Name;                // 定義名
    String Filename;            // glTFファイル名
    Float3 Trans;               // 物体の座標(オフセット)
    Float3 Scale;               // 物体の大きさ

    Float3 Rotate[2];           // 物体の向き(オイラー角)  [0]メイン [1]サブ
    Float3 Spin[2];             // 物体の回転量(オイラー角)[0]メイン [1]サブ
    uint32  Attr;               // 適用する属性レイヤの番号 0：影響なし
    float  Dummy;
    Quaternion qRotate[2];      // 物体の向き(四元数)      [0]メイン [1]サブ
    Quaternion qSpin[2];        // 物体の回転量(四元数)    [0]メイン [1]サブ

    String Mode;                // モード[連番アニメ｜乱数選択]
    float ArgF;                 // 小数引数 [ 文字の場合はKerning値 | アニメ付の場合再生速度 ]
    int32 ArgI;                 // 整数引数 [ 乱数モードフレーム番号 | アニメ付の場合アニメ番号 ] 
    Array<NoAModel> NoAModels;  // アニメ無しモデル配列(連番glTF)
    AnimeModel      AniModel;   // アニメ付きモデル
};

//ウェイトテーブルのデフォルト設定
#define FRAME 16
#define RE(NN) ((float)NN/(float)FRAME)
Array<float> WEIGHTTRANS = { RE(0),RE(1),RE(2),RE(3) ,RE(4) ,RE(5), RE(6), RE(7), RE(8), RE(9), RE(10),RE(11),RE(12),RE(13),RE(14),RE(15),-1 };

Array<float> WEIGHTBLINK = { RE(0),RE(1),RE(2),RE(3) ,RE(4) ,RE(5), RE(6), RE(7), RE(8), RE(9), RE(10),RE(11),RE(12),RE(13),RE(14),RE(15),
                             RE(14),RE(13),RE(12),RE(11) ,RE(10) ,RE(9), RE(8), RE(7), RE(6), RE(5), RE(4),RE(3),RE(2),RE(1),RE(0),-1 };

//モーフ情報
struct MorphInfo
{
    float Speed;                // モーフ速度
    float CntSpeed;             // モーフ速度制御カウント(>1.0で更新)
    int32 NowTarget;            // モーフターゲット(現在)  無効-1、通常0～
    int32 DstTarget;            // モーフターゲット(遷移先)通常-1、設定あれば0～
    int32 IndexTrans;           // 遷移カウント、通常-1、遷移中0～
    Array<float> WeightTrans;   // 遷移ウェイトテーブル、-1か100％で終端
};


//物体情報コンテナ
struct Entity
{
    String Type;                // 物体タイプ[物:静物 | 敵:敵MOB | 主:プレーヤー]
    String Name;                // 定義名                 ※現在のインスタンス用（不要？）　
    uint32 Frame;               // フレーム番号(開始)     ※現在のインスタンス用　
    MapleInfo* Maple;           // 記号情報               ※記号情報へのアクセス

    Float3 Trans;               // 物体の座標        　   ※現在のインスタンス用　
    Float3 Scale;               // 物体の大きさ　　　　　 ※現在のインスタンス用　
    Float3 Rotate[2];           // 物体の向き(オイラー角) ※現在のインスタンス用      
    Quaternion qRotate[2];      // 物体の向き(四元数) 　　※現在のインスタンス用[0:文字角度| 1:文字列角度]     

    float  Start;               // 表示開始位置
    float  Count;               // 表示量

    MorphInfo Morph[NUMM];      // モーフィング制御 ※3ch合成

    Array<int32> CtrlI;
    Array<float> CtrlF;

};

struct MapleMap
{
    Array<Attribute> Attribs;      // 属性の座標リスト

    Array<Entity> Entities;        // 物体の座標リスト
    Array<MapleInfo> Maples;       // Maple設定は独立の配列としておいてエンティティから参照する
};


void gltfCalcSkeleton(tinygltf::Model& model, tinygltf::Node& node, Mat4x4& matparent);

tinygltf::Buffer* getBuffer(tinygltf::Model& gltfmodel, tinygltf::Primitive& pr, int* offset, int* stride)
{
    if (pr.indices == -1) return nullptr;
    auto& ai = gltfmodel.accessors[pr.indices];
    auto& bi = gltfmodel.bufferViews[ai.bufferView];
    auto& buf = gltfmodel.buffers[bi.buffer];
    *offset = bi.byteOffset + ai.byteOffset;
    *stride = ai.ByteStride(bi);
    return &buf;
}

tinygltf::Buffer* getBuffer(tinygltf::Model& gltfmodel, tinygltf::Primitive& pr, const std::string attr, int* offset, int* stride)
{
    if (pr.attributes.size() == 0) return nullptr;
    auto& ap = gltfmodel.accessors[pr.attributes[attr]];
    auto& bp = gltfmodel.bufferViews[ap.bufferView];
    auto& buf = gltfmodel.buffers[bp.buffer];
    *offset = bp.byteOffset + ap.byteOffset;
    *stride = ap.ByteStride(bp);
    return &buf;
}

static tinygltf::Buffer* getBuffer(tinygltf::Model& model, tinygltf::Primitive& pr, int32 morphtarget, const char* attr, int* offset, int* stride)
{
    if (pr.targets.size() == 0) return nullptr;
    auto& at = model.accessors[pr.targets[morphtarget].at(attr)];
    auto& bt = model.bufferViews[at.bufferView];
    auto& buf = model.buffers[bt.buffer];
    *offset = bt.byteOffset + at.byteOffset;
    *stride = at.ByteStride(bt);
    return &buf;
}

template <typename T> tinygltf::Value toVal(T value)
{
    return tinygltf::Value((std::vector<unsigned char>) reinterpret_cast<std::vector<unsigned char>&>(value));
}

tinygltf::Value toVal(Float4* value, size_t size)
{
    return tinygltf::Value(reinterpret_cast<const unsigned char*>(value), size);
}

tinygltf::Value toVal(Mat4x4* value, size_t size)
{
    return tinygltf::Value(reinterpret_cast<const unsigned char*>(value), size);
}

template <typename T> std::vector<float> toFloat(T value)
{
    return reinterpret_cast<std::vector<float>&>(value.Get<std::vector<unsigned char>>());
}

Mat4x4 toMat(tinygltf::Value value)
{
    Mat4x4 mat = Mat4x4().Identity();
    if (value.IsBinary())
        std::memcpy(&mat, reinterpret_cast<void*>(value.Get<std::vector<unsigned char>>().data()), sizeof(Mat4x4));
    return mat;
}

void gltfSetupPosture(tinygltf::Model& gltfmodel, tinygltf::Node& node)
{
    node.extensions["poserot"] = toVal(&Float4(0, 0, 0, 1), sizeof(Float4));
    node.extensions["posetra"] = toVal(&Float4(0, 0, 0, 0), sizeof(Float4));
    node.extensions["posesca"] = toVal(&Float4(1, 1, 1, 0), sizeof(Float4));
    node.extensions["posewei"] = toVal(&Float4(0, 0, 0, 0), sizeof(Float4));

    auto& r = node.rotation;
    auto& t = node.translation;
    auto& s = node.scale;

    Quaternion rr = r.size() ? Quaternion(r[0], r[1], r[2], r[3]) : Quaternion(0, 0, 0, 1);
    Float3     tt = t.size() ? Float3(t[0], t[1], t[2]) : Float3(0, 0, 0);
    Float3     ss = s.size() ? Float3(s[0], s[1], s[2]) : Float3(1, 1, 1);

    Mat4x4 matlocal = Mat4x4().Identity().Scale(ss) * rr.toMatrix() * Mat4x4().Identity().Translate(tt);
    node.extensions["matlocal"] = toVal(&matlocal, sizeof(Mat4x4));

    //子ノードを全て基本姿勢登録
    for (uint32 cc = 0; cc < node.children.size(); cc++)
        gltfSetupPosture(gltfmodel, gltfmodel.nodes[node.children[cc]]);
}


void gltfSetupMesh(NoAModel& noamodel, tinygltf::Model& gltfmodel, tinygltf::Node& node)
{
    for (int32 pp = 0; pp < gltfmodel.meshes[node.mesh].primitives.size(); pp++)
    {
        //準備
        auto& pr = gltfmodel.meshes[node.mesh].primitives[pp];
        auto& map = gltfmodel.accessors[pr.attributes["POSITION"]];

        int32 opos = 0, otex = 0, onormal = 0, ojoints = 0, oweights = 0, oidx = 0, stride = 0, texstride = 0;

        auto& bpos = *getBuffer(gltfmodel, pr, "POSITION", &opos, &stride);
        auto& btex = *getBuffer(gltfmodel, pr, "TEXCOORD_0", &otex, &texstride);
        auto& bnormal = *getBuffer(gltfmodel, pr, "NORMAL", &onormal, &stride);
        auto& bjoint = *getBuffer(gltfmodel, pr, "JOINTS_0", &ojoints, &stride);
        auto& bweight = *getBuffer(gltfmodel, pr, "WEIGHTS_0", &oweights, &stride);
        auto& bidx = *getBuffer(gltfmodel, pr, &oidx, &stride);

        //頂点座標を生成
        Array<MeshVertex> vertices;
        for (int32 vv = 0; vv < map.count; vv++)
        {
            float* vertex = nullptr, * texcoord = nullptr, * normal = nullptr;
            vertex = (float*)&bpos.data.at(vv * 12 + opos);
            texcoord = (float*)&btex.data.at(vv * 8 + otex);
            normal = (float*)&bnormal.data.at(vv * 12 + onormal);

            MeshVertex mv;
            mv.position = Float3(vertex[0], vertex[1], vertex[2]);
            mv.texcoord = Float2(texcoord[0], texcoord[1]);
            mv.normal = Float3(normal[0], normal[1], normal[2]);
            if (pr.attributes["TEXCOORD_0"]) mv.texcoord = Float2(texcoord[0], texcoord[1]);

            Mat4x4 matskin = toMat(node.extensions["matlocal"]);
            auto matpos = matskin.transform(Float4(mv.position, 1.0f));
            auto matnor = matskin.inverse().transposed();
            mv.position = Float3(matpos.x, matpos.y, matpos.z) / matpos.w;
            mv.normal = matnor.transform(mv.normal);

#ifdef CPUSKINNING
            //CPUスキニング
            if (gltfmodel.skins.size() >= 0 && pr.attributes["JOINTS_0"] && pr.attributes["WEIGHTS_0"])
            {
                Mat4x4 matskin = Mat4x4().Identity();

                auto joint = (uint16_t*)&bjoint.data.at(vv * 8 + ojoints);//1頂点あたり4JOINT
                auto weight = (float*)&bweight.data.at(vv * 16 + oweights);
                Word4  j4 = Word4(joint[0], joint[1], joint[2], joint[3]);
                Float4 w4 = Float4(weight[0], weight[1], weight[2], weight[3]);

                //4ジョイント合成
                matskin = w4.x * noamodel.Joints[node.skin][j4.x] +
                          w4.y * noamodel.Joints[node.skin][j4.y] +
                          w4.z * noamodel.Joints[node.skin][j4.z] +
                          w4.w * noamodel.Joints[node.skin][j4.w];

                auto matpos = matskin.transform(Float4(mv.position, 1.0f));
                auto matnor = matskin.inverse().transposed();

                mv.position = Float3(matpos.x, matpos.y, matpos.z) / matpos.w;
                mv.normal = matnor.transform(mv.normal);

            }
#endif //CPUSKINNING
            if (pr.targets.size() > 0)
            {
                noamodel.morphMatBuffer.emplace_back(matskin);
                noamodel.morphMesh.TexCoord.emplace_back(mv.texcoord);
                noamodel.morphMesh.TexCoordCount = 0;//未使用
            }

            vertices.emplace_back(mv);
        }

        //頂点インデクスを生成してメッシュ生成
        MeshData md;

        if (pr.indices > 0)
        {
            auto& mapi = gltfmodel.accessors[pr.indices];
            Array<uint32_t> indices;
            for (int32 i = 0; i < mapi.count; i++)
            {
                uint32_t idx = 0;
                idx = (mapi.componentType == 5123) ? *(uint16_t*)&bidx.data.at(i * 2 + oidx) : //16bit
                    (mapi.componentType == 5125) ? *(uint32_t*)&bidx.data.at(i * 4 + oidx) : 0;//32bit
                indices.emplace_back(idx);
            }

            md = MeshData(vertices, indices);

            vertices.clear();
            indices.clear();

        }

        //頂点色とテクスチャを登録
        auto texcol = 0;// テクスチャ頂点色識別子 b0=頂点色 b1=テクスチャ
        Texture tex;
        ColorF col = ColorF(1, 1, 1, 1);
        if (pr.material >= 0)
        {
            auto& nt = gltfmodel.materials[pr.material].additionalValues["normalTexture"];  //法線マップ
            int32 idx = -1;
            auto& mmv = gltfmodel.materials[pr.material].values;
            auto& bcf = mmv["baseColorFactor"];                                         //色
            if (mmv.count("baseColorTexture"))
                idx = gltfmodel.textures[(int)mmv["baseColorTexture"].json_double_value["index"]].source;

            //頂点色を登録
            if (bcf.number_array.size()) col = ColorF(bcf.number_array[0], bcf.number_array[1], bcf.number_array[2], bcf.number_array[3]);

            //テクスチャを登録
            if (idx >= 0 && gltfmodel.images.size())
            {
                tex = Texture();
                if (gltfmodel.images[idx].bufferView >= 0)
                {
                    auto& bgfx = gltfmodel.bufferViews[gltfmodel.images[idx].bufferView];
                    auto bimg = &gltfmodel.buffers[bgfx.buffer].data.at(bgfx.byteOffset);

                    ByteArray teximg((void*)bimg, bgfx.byteLength);
                    tex = Texture(std::move(teximg), TextureDesc::For3D);
                }
                else
                {
                    auto& mii = gltfmodel.images[idx].image;
                    ByteArray teximg((void*)&mii, mii.size());
                    tex = Texture(std::move(teximg), TextureDesc::For3D);
                }
                texcol |= 2;
            }
            texcol |= 1;
        }

        if (texcol & 2) noamodel.meshTexs.emplace_back(tex);                // テクスチャ追加
        noamodel.meshColors.emplace_back(col);                              // 頂点色追加
        noamodel.meshName.emplace_back(FromUTF8(gltfmodel.meshes[node.mesh].name));  // ノード名追加
        noamodel.Meshes.emplace_back(DynamicMesh(md));                      // メッシュ追加
        noamodel.TexCol.emplace_back(texcol);                               // テクスチャ頂点色識別子追加
    }
}

//モーフありメッシュは動作時にジオメトリ変換必要なので、事前準備がまるで無駄に
// ①モーフありメッシュの判別：morphTargets[]
// ②モーフ形状バッファmorphBuffer[][]
// ③モーフ基本形状のコピーmorphBasisBuffer[][]※②+③が動作時に必要
// ④モーフ有メッシュのUV座標数(Siv3DはDynamicMeshにすると参照ができないので、再構築のために事前に持っておく必要ある)
// ⑤モーフ有メッシュのUV座標morphTexCoord

void gltfSetupMorph(tinygltf::Model& gltfmodel, tinygltf::Node& node, MorphMesh& morph)
{
    for (int32 pp = 0; pp < gltfmodel.meshes[node.mesh].primitives.size(); pp++)
    {
        auto& pr = gltfmodel.meshes[node.mesh].primitives[pp];
        morph.Targets.emplace_back((int32)pr.targets.size()); //モーフィング有無追加(15個または0個)

        if (pr.targets.size() > 0)
        {

            //Meshes->Primitive->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
            Array<MeshVertex> vertices;

            int32 offsetpos = 0, offsetnor = 0, stride = 0;
            auto basispos = getBuffer(gltfmodel, pr, "POSITION", &offsetpos, &stride);
            auto basisnor = getBuffer(gltfmodel, pr, "NORMAL", &offsetnor, &stride);


            //元メッシュ(basis)の頂点座標バッファを生成
            auto& numvertex = gltfmodel.accessors[pr.attributes["POSITION"]].count; //元メッシュの頂点/法線数
            for (int32 vv = 0; vv < numvertex; vv++)
            {
                MeshVertex mv;
                auto pos = (float*)&basispos->data.at(vv * 12 + offsetpos);
                auto nor = (float*)&basisnor->data.at(vv * 12 + offsetnor);
                mv.position = Float3(pos[0], pos[1], pos[2]);
                mv.normal = Float3(nor[0], nor[1], nor[2]);
                vertices.emplace_back(mv);
            }

            morph.BasisBuffer.emplace_back(vertices);
            vertices.clear();

            //シェイプキー(複数)の頂点座標バッファを生成
            for (int32 tt = 0; tt < pr.targets.size(); tt++)
            {
                //Meshes->Primitive->Target->POSITION->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
                int32 offsetpos = 0, offsetnor = 0, stride = 0;
                auto mtpos = getBuffer(gltfmodel, pr, tt, "POSITION", &offsetpos, &stride);
                auto mtnor = getBuffer(gltfmodel, pr, tt, "NORMAL", &offsetnor, &stride);

                Array<MeshVertex> vertices;
                //モーフターゲットメッシュの頂点座標バッファを生成
                for (int32 vv = 0; vv < numvertex; vv++)
                {
                    MeshVertex mv;
                    auto pos = (float*)&mtpos->data.at(vv * 12 + offsetpos);
                    auto nor = (float*)&mtnor->data.at(vv * 12 + offsetnor);
                    mv.position = Float3(pos[0], pos[1], pos[2]);
                    mv.normal = Float3(nor[0], nor[1], nor[2]);
                    vertices.emplace_back(mv);
                }
                morph.Buffers.emplace_back(vertices);
                vertices.clear();
            }
        }
    }
}

void gltfSetupModel(NoAModel& noamodel, tinygltf::Model& gltfmodel)
{
    for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
    {
        auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];

        //msn直下にメッシュある場合
        if (msn.mesh >= 0) gltfSetupPosture(gltfmodel, msn);

        //子ノードを再起で基本姿勢登録
        for (int32 cc = 0; cc < msn.children.size(); cc++)
            gltfSetupPosture(gltfmodel, gltfmodel.nodes[msn.children[cc]]);
    }

#ifdef CPUSKINNING
    for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
    {
        Mat4x4 ident = Mat4x4().Identity();
        auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
        if (msn.mesh >= 0)
            gltfCalcSkeleton(gltfmodel, msn, ident);

        for (int32 cc = 0; cc < msn.children.size(); cc++)
        {
            auto& node = gltfmodel.nodes[msn.children[cc]];
            gltfCalcSkeleton(gltfmodel, node, ident);
        }
    }

    //CPUスキニング
    if (gltfmodel.skins.size() >= 0)
    {
        noamodel.Joints.resize(gltfmodel.skins.size());
        for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
        {
            auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
            for (int32 cc = 0; cc < msn.children.size(); cc++)
            {
                auto& node = gltfmodel.nodes[msn.children[cc]];
                if (node.skin < 0) continue;         //LightとCameraをスキップ

                auto& msns = gltfmodel.skins[node.skin];
                auto& ibma = gltfmodel.accessors[msns.inverseBindMatrices];
                auto& ibmbv = gltfmodel.bufferViews[ibma.bufferView];
                auto  ibmd = gltfmodel.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

                //Meshを参照するノードはskinを参照する
                for (int32 ii = 0; ii < msns.joints.size(); ii++)
                {
                    Mat4x4 matinverse = Mat4x4().Identity();        //この記述は適当
                    Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
                    Mat4x4 matworld = toMat(gltfmodel.nodes[msns.joints[ii]].extensions["matworld"]);
                    Mat4x4 matjoint = ibm * matworld * matinverse;

                    noamodel.Joints[node.skin].emplace_back(matjoint);
                }
            }
        }
    }

#endif //CPUSKINNING

    for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
    {
        auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
        if (msn.mesh >= 0)
        {
            gltfSetupMesh(noamodel, gltfmodel, msn);
            gltfSetupMorph(gltfmodel, msn, noamodel.morphMesh);
        }

        for (int32 cc = 0; cc < msn.children.size(); cc++)
        {
            auto& node = gltfmodel.nodes[msn.children[cc]];
            if (node.mesh >= 0)
            {
                gltfSetupMesh(noamodel, gltfmodel, node);
                gltfSetupMorph(gltfmodel, node, noamodel.morphMesh);
            }
        }
    }
}

void gltfDrawMesh(NoAModel* noamodel, Entity& entity, Quaternion& rot, Float3& tra, Float3& sca,
    size_t istart = 0, size_t icount = -1)  //筆順表現用引数
{
    int32 cntm = 0;                     //モーフありメッシュの個数
    int32 tid = 0;
    for (int32 mm = 0; mm < noamodel->Meshes.size(); mm++)
    {
        auto& morphs = noamodel->morphMesh.Targets[mm];
        if (morphs > 0)  //モーフあり
        {
            Array<MeshVertex> morphmv = noamodel->morphMesh.BasisBuffer[cntm]; //元メッシュのコピーを作業用で確保
            auto& buf = noamodel->morphMesh.Buffers;

            for (auto vv = 0; vv < morphmv.size(); vv++)
            {
                for (auto ii = 0; ii < NUMM; ii++)   //3chモーフ合成(目、口、表情を非同期で動かす)
                {
                    auto& now = entity.Morph[ii].NowTarget;
                    auto& dst = entity.Morph[ii].DstTarget;
                    auto& idx = entity.Morph[ii].IndexTrans;
                    auto& wt = entity.Morph[ii].WeightTrans;

                    if (now == -1) continue;    //NowTargetが-1の場合はモーフ無効
                    if (idx == -1)             //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
                    {
                        auto weight = entity.Morph[0].WeightTrans;
                        morphmv[vv].position += buf[cntm * NUMM + ii][vv].position * (1 - wt[0]) + buf[cntm * NUMM + ii][vv].position * wt[0];
                        morphmv[vv].normal += buf[cntm * NUMM + ii][vv].normal * (1 - wt[0]) + buf[cntm * NUMM + ii][vv].normal * wt[0];
                    }
                    else
                    {
                        auto weight = (wt[idx] < 0) ? 0 : wt[idx];
                        morphmv[vv].position += buf[cntm * NUMM + now][vv].position * (1 - weight) + buf[cntm * NUMM + dst][vv].position * weight;
                        morphmv[vv].normal += buf[cntm * NUMM + now][vv].normal * (1 - weight) + buf[cntm * NUMM + dst][vv].normal * weight;
                    }
                }
                Mat4x4& matskin = noamodel->morphMatBuffer[vv];
                auto matpos = matskin.transform(Float4(morphmv[vv].position, 1.0f)); //モーフィング部分のみ動的スキニング
                auto matnor = matskin.inverse().transposed();
                morphmv[vv].position = Float3(matpos.x, matpos.y, matpos.z) / matpos.w;
                morphmv[vv].normal = matnor.transform(morphmv[vv].normal);
                morphmv[vv].texcoord = noamodel->morphMesh.TexCoord[vv];
            }
            auto dm = noamodel->Meshes[mm];
            dm.fillVertices(morphmv);

            auto& color = noamodel->meshColors[mm];
            if (noamodel->TexCol[mm] & 2)  dm.rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, noamodel->meshTexs[tid++]);
            else                           dm.rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, color);
            cntm++;
        }

        else           //モーフなし
        {
            auto& mesh = noamodel->Meshes[mm].rotated(rot).scaled(sca).translated(tra);
            auto& color = noamodel->meshColors[mm];

            if (noamodel->TexCol[mm] & 2) mesh.drawSubset(istart, icount, noamodel->meshTexs[tid++]);
            else                          mesh.drawSubset(istart, icount, color);
        }
    }
}


//                     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
const uint32 A2M[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //00 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //10 
                       0,35,36,37,38,39,40,41,42,43,53,55,60,47,61,93, //20  !"#$
                      25,26,27,28,29,30,34,31,32,33,94,54,58,44,59,56, //30 01234
                      64,90, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13, //40 @ABCD
                      14,15,16,17,18,19,20,21,22,23,24,62,49,63,48,57, //50 PQRST
                      50,91,65,66,67,68,69,70,71,72,73,74,75,76,77,78, //60 'abcd
                      79,80,81,82,83,84,85,86,87,88,89,90,51,46,52,45, //70 pqrst
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //80 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //90 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //A0 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //B0 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //C0 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //D0 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //E0 
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//F0 

void gltfDrawString(NoAModel* model, String text, float kerning, float radius,
    Quaternion* qrotate, Float3& trans, Float3& scale,
    uint32 istart = 0, float icount = 0)
{
    text += L" ";   //始端と終端の筆順表現が難しいので終端に空白挿入で対応

    auto isall = false;
    if (icount < 0)  //icountが負の場合は全体筆順表現
    {
        icount = -icount;
        isall = true;
    }

    icount = (icount > text.length) ? text.length : icount;
    uint32 start = (istart > text.length) ? text.length : istart;
    start += 2;             //文:<文字列>
    uint32 len = icount;
    float  stroke = icount - (uint32)icount;
    len = ((start + len) > text.length) ? text.length : start + len;

    if (text.substr(0, 1) == L"線")
    {
        auto i = start;
        Float3 tt = trans;
        // auto tt = qrotate[1] * Vec3(radius, 0, 0);                                        //座標
        Float3 kvec3 = qrotate[0].toMatrix().transform(Float3(kerning * (scale.x / 50), 0, 0));

        for (; i < len; i++)
        {
            auto ascii = text.substr(i, 1).narrow().c_str()[0];
            auto& chr = A2M[ascii];

            if (!chr || ascii == ' ')
            {
                tt += kvec3;
                continue;
            }

            if (isall)
            {
                auto count = model->Meshes[chr].num_indices;
                count = (count + 10) * stroke > count ? count : (count + 40) * stroke; //10カウント下駄をはかせて描画 
                auto& mesh = model->Meshes[chr].rotated(qrotate[0]).scaled(scale).translated(tt);
                auto& color = model->meshColors[chr];
                mesh.drawSubset(0, count, color);
                tt += kvec3;
            }
            else
            {
                if (i < (len - 1))
                {
                    auto& mesh = model->Meshes[chr].rotated(qrotate[0]).scaled(scale).translated(tt);
                    auto& color = model->meshColors[chr];
                    mesh.draw(color);
                    tt += kvec3;
                }
                else
                {
                    auto& mesh = model->Meshes[chr].rotated(qrotate[0]).scaled(scale).translated(tt);
                    auto& color = model->meshColors[chr];
                    mesh.drawSubset(0, model->Meshes[chr].num_indices * stroke, color);
                }
            }
        }
    }

    else if (text.substr(0, 1) == L"円")
    {
        auto ii = start;

        for (; ii < len; ii++)
        {
            auto ascii = text.substr(ii, 1).narrow().c_str()[0];
            auto& chr = A2M[ascii];

            if (!chr || ascii == ' ') continue;

            auto q0001 = Quaternion(0, 0, 0, 1);
            Quaternion r = q0001.rotateRollPitchYaw(Radians(0), Radians(0), Radians(ii * kerning));//向き
            auto tt = (r * qrotate[1]) * Vec3(radius, 0, 0);                                      //座標

            if (isall)
            {
                auto count = model->Meshes[chr].num_indices;
                count = (count + 10) * stroke > count ? count : (count + 40) * stroke; //10カウント下駄をはかせて描画 
                auto& mesh = model->Meshes[chr].rotated(qrotate[0] * r).scaled(scale).translated(trans + tt);
                auto& color = model->meshColors[chr];
                mesh.drawSubset(0, count, color);
            }
            else
            {
                if (ii < (len - 1))
                {
                    auto& mesh = model->Meshes[chr].rotated(qrotate[0] * r).scaled(scale).translated(trans + tt);
                    auto& color = model->meshColors[chr];
                    mesh.draw(color);
                }
                else
                {
                    auto& mesh = model->Meshes[chr].rotated(qrotate[0] * r).scaled(scale).translated(trans + tt);
                    auto& color = model->meshColors[chr];
                    mesh.drawSubset(0, model->Meshes[chr].num_indices * stroke, color);
                }
            }
        }
    }
}

// --- --- --- --- 

void gltfInterpolateStep(tinygltf::Model& model, Channel& ch, int32 lowframe);
void gltfInterpolateLinear(tinygltf::Model& model, Channel& ch, int32 lowframe, int32 uppframe, float weight);
void gltfInterpolateSpline(tinygltf::Model& model, Channel& ch, int32 lowframe, int32 uppframe, float lowtime, float upptime, float weight);
void gltfPrecomputeMesh(tinygltf::Model& model, int32 idxframe, AnimeModel& animodel, int32 aid, tinygltf::Node& node);
void gltfSetupAnime(AnimeModel& animodel, tinygltf::Model& model, int32 aid, int32 interpolate);

//void gltfSetupModel( AnimeModel& animodel, tinygltf::Model& gltfmodel, int32 anime)
void gltfSetupModel( MapleInfo &mi, tinygltf::Model& gltfmodel, int32 anime)
{
    //シーンに含まれるノード(モデル)を検索してchildrenがあるノードをモデルルートとする。無いノードは光源とカメラ。
    for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
    {
        auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
        //msn直下にメッシュある場合
        if (msn.mesh >= 0) gltfSetupPosture(gltfmodel, msn);

        //子ノードを再起で基本姿勢登録
        for (int32 cc = 0; cc < msn.children.size(); cc++)
            gltfSetupPosture(gltfmodel, gltfmodel.nodes[msn.children[cc]]);
    }

    mi.AniModel.precAnimes.resize(gltfmodel.animations.size());
    mi.AniModel.morphMesh.TexCoordCount = (unsigned)-1;

    if (anime == -1)                                //全アニメデコード対象
    {
        for (auto aid = 0; aid < gltfmodel.animations.size(); aid++)
            gltfSetupAnime( mi.AniModel, gltfmodel, aid, 1);
    }
    else
        gltfSetupAnime( mi.AniModel, gltfmodel, anime, 1); //1アニメデコード対象
}

void gltfSetupAnime(AnimeModel& animodel, tinygltf::Model& gltfmodel, int32 aid, int32 interpolate)
{
    auto& man = gltfmodel.animations[aid];
    auto& mas = man.samplers;
    auto& mac = man.channels;
    auto& macc = gltfmodel.accessors;
    auto& mbv = gltfmodel.bufferViews;
    auto& mb = gltfmodel.buffers;

    auto cycleframe = man.channels.size()*interpolate;         // ループアニメ1周のフレーム数

    auto begin = macc[mas[0].input].minValues[0];               // 現在時刻
    auto end = macc[mas[0].input].maxValues[0];                 // ループアニメ周期（アニメーション時間の最大はSamplerInputの最終要素に記録されてる）
    auto frametime = (end - begin) / cycleframe;                // 1フレーム時間
    auto currenttime = begin;
    auto cycletime = end;

    //全フレームの補間形式(step,linear,3dspline)、フレーム時刻、delta(移動,回転,拡縮,ウェイト)を収集
    animodel.precAnimes[aid].Frames.resize(cycleframe);
    for (int32 cf = 0; cf < cycleframe; cf++)
    {
        auto& as = animodel.precAnimes[aid].Frames[cf].Samplers;
        auto& ac = animodel.precAnimes[aid].Frames[cf].Channels;
        as.resize(mas.size());
        ac.resize(mac.size());

        //モーフメッシュのTexCoordを初回アニメの初回フレームのみで収集するので制限数を設定
        if (cf == 1 && animodel.morphMesh.TexCoordCount == (unsigned)-1)
            animodel.morphMesh.TexCoordCount = animodel.morphMesh.TexCoord.size();

        // GLTFサンプラを取得
        for (int32 ss = 0; ss < mas.size(); ss++)
        {
            auto& masi = gltfmodel.accessors[mas[ss].input];  // 前フレーム情報

            as[ss].minTime = 0;
            as[ss].maxTime = 1;
            if (masi.minValues.size() > 0 && masi.maxValues.size() > 0)
            {
                as[ss].minTime = float(masi.minValues[0]);
                as[ss].maxTime = float(masi.maxValues[0]);
            }

            as[ss].interpolateKeyframes.resize(masi.count);

            for (auto kk = 0; kk < masi.count; kk++)
            {
                auto& sai = mas[ss].input;
                const auto& offset = mbv[macc[sai].bufferView].byteOffset + macc[sai].byteOffset;
                const auto& stride = masi.ByteStride(mbv[masi.bufferView]);
                void* adr = &mb[mbv[macc[sai].bufferView].buffer].data.at(offset + kk * stride);

                auto& ctype = macc[sai].componentType;
                float value = (ctype == 5126) ? *(float*)adr :
                    (ctype == 5123) ? *(uint16_t*)adr :
                    (ctype == 5121) ? *(uint8_t*)adr :
                    (ctype == 5122) ? *(int16_t*)adr :
                    (ctype == 5120) ? *(int8_t*)adr : 0.0;

                as[ss].interpolateKeyframes[kk] = std::make_pair(kk, value);
            }
        }

        // GLTFチャネルを取得
        for (int32 cc = 0; cc < ac.size(); cc++)
        {
            auto& maso = gltfmodel.accessors[mas[cc].output];
            auto& macso = macc[mas[mac[cc].sampler].output];
            const auto& stride = maso.ByteStride(mbv[maso.bufferView]);
            const auto& offset = mbv[macso.bufferView].byteOffset + macso.byteOffset;

            ac[cc].deltaKeyframes.resize(maso.count);
            ac[cc].idxNode = mac[cc].target_node;
            ac[cc].idxSampler = mac[cc].sampler;

            if (mac[cc].target_path == "weights")
            {
                ac[cc].typeDelta = 4;   //weight

                for (int32 ff = 0; ff < maso.count; ff++)
                {
                    void* adr = (void*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);

                    auto& ctype = macso.componentType;
                    float value = (ctype == 5126) ? *(float*)adr :
                        (ctype == 5123) ? *(uint16_t*)adr :
                        (ctype == 5121) ? *(uint8_t*)adr :
                        (ctype == 5122) ? *(int16_t*)adr :
                        (ctype == 5120) ? *(int8_t*)adr : 0.0;

                    ac[cc].deltaKeyframes[ff].first = ff;
                    ac[cc].deltaKeyframes[ff].second = Float4(value, 0, 0, 0);
                }
            }

            if (mac[cc].target_path == "translation")
            {
                ac[cc].typeDelta = 1;   //translate

                for (int32 ff = 0; ff < maso.count; ff++)
                {
                    float* tra = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
                    ac[cc].deltaKeyframes[ff].first = ff;
                    ac[cc].deltaKeyframes[ff].second = Float4(tra[0], tra[1], tra[2], 0);
                }
            }

            if (mac[cc].target_path == "rotation")
            {
                ac[cc].typeDelta = 3;
                for (int32 ff = 0; ff < maso.count; ff++)
                {
                    float* rot = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
                    auto qt = Quaternion(rot[0], rot[1], rot[2], rot[3]).normalize();

                    ac[cc].deltaKeyframes[ff].first = ff;
                    ac[cc].deltaKeyframes[ff].second = Float4(qt.component.m128_f32[0],
                        qt.component.m128_f32[1],
                        qt.component.m128_f32[2],
                        qt.component.m128_f32[3]);
                }
            }

            if (mac[cc].target_path == "scale")
            {
                ac[cc].typeDelta = 2;
                for (int32 ff = 0; ff < maso.count; ff++)
                {
                    float* sca = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
                    ac[cc].deltaKeyframes[ff].first = ff;
                    ac[cc].deltaKeyframes[ff].second = Float4(sca[0], sca[1], sca[2], 0);
                }
            }
        }

        // 姿勢確定
        for (auto& ch : animodel.precAnimes[aid].Frames[cf].Channels)
        {
            auto& sa = animodel.precAnimes[aid].Frames[cf].Samplers[ch.idxSampler];

            std::pair<int, float> f0, f1;
            size_t kf;
            for (kf = 0; kf < sa.interpolateKeyframes.size() - 1; kf++)
            {
                f0 = sa.interpolateKeyframes[kf];
                f1 = sa.interpolateKeyframes[kf + 1];
                if (f0.second <= currenttime && f1.second >= currenttime) break;
            }
            if (kf == sa.interpolateKeyframes.size()) break;

            float lowtime = f0.second;
            float upptime = f1.second;
            const int32 lowframe = f0.first;
            const int32 uppframe = f1.first;
            auto& interpol = mas[ch.idxSampler].interpolation;

            // 再生時刻を正規化してキーフレーム間ウェイト算出
            const float mix = (currenttime - lowtime) / (upptime - lowtime);

            //キーフレーム間ウェイト、補間モード、下位フレーム/時刻、上位フレーム/時刻から姿勢確定
            if (interpol == "STEP")             gltfInterpolateStep  (gltfmodel, ch, lowframe);
            else if (interpol == "LINEAR")      gltfInterpolateLinear(gltfmodel, ch, lowframe, uppframe, mix);
            else if (interpol == "CUBICSPLINE") gltfInterpolateSpline(gltfmodel, ch, lowframe, uppframe, lowtime, upptime, mix);
        }

        for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
        {
            auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
            for (int32 cc = 0; cc < msn.children.size(); cc++)
            {
                auto& node = gltfmodel.nodes[msn.children[cc]];
                gltfCalcSkeleton(gltfmodel, node, Mat4x4().Identity());
            }
        }

        Mat4x4 matinverse = Mat4x4().Identity();        //この記述は適当

        animodel.precAnimes[aid].Joints.clear();
        animodel.precAnimes[aid].Joints.resize(gltfmodel.skins.size());

        for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
        {
            auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
            for (int32 cc = 0; cc < msn.children.size(); cc++)
            {
                auto& node = gltfmodel.nodes[msn.children[cc]];
                if (node.skin < 0) continue;         //LightとCameraをスキップ

                auto& msns = gltfmodel.skins[node.skin];
                auto& ibma = gltfmodel.accessors[msns.inverseBindMatrices];
                auto& ibmbv = gltfmodel.bufferViews[ibma.bufferView];
                auto  ibmd = gltfmodel.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

                //Meshを参照するノードはskinを参照する
                for (int32 ii = 0; ii < msns.joints.size(); ii++)
                {
//                    _LOG(L"** msns=%d", ii);

                    auto& skeletonode = gltfmodel.nodes[msns.joints[ii]];
                    Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
                    Mat4x4 matworld = toMat(skeletonode.extensions["matworld"]);
                    Mat4x4 matjoint = ibm * matworld * matinverse;

                    animodel.precAnimes[aid].Joints[node.skin].emplace_back(matjoint);
                }
            }
        }

        for (int32 nn = 0; nn < gltfmodel.scenes[0].nodes.size(); nn++)
        {
            auto& msn = gltfmodel.nodes[gltfmodel.scenes[0].nodes[nn]];
            for (int32 cc = 0; cc < msn.children.size(); cc++)
            {
                auto& node = gltfmodel.nodes[msn.children[cc]];
                if (node.mesh >= 0)//メッシュノード　
                {
                    //現在のノード(メッシュ)を描画(前計算)
                    gltfPrecomputeMesh(gltfmodel, cf, animodel, aid, node);

                    if (cf == 0) //最初のフレームでモーフィング情報取得
                        gltfSetupMorph(gltfmodel, node, animodel.morphMesh);
                }
            }
            currenttime += frametime;   //次のフレーム時刻に進める
        }
    }
}

void gltfInterpolateStep(tinygltf::Model& model, Channel& ch, int32 lowframe)
{
    Float4 val = ch.deltaKeyframes[lowframe].second;
    if (ch.typeDelta == 1)      model.nodes[ch.idxNode].extensions["posetra"] = toVal(&val);
    else if (ch.typeDelta == 3) model.nodes[ch.idxNode].extensions["poserot"] = toVal(&val);
    else if (ch.typeDelta == 2) model.nodes[ch.idxNode].extensions["posesca"] = toVal(&val);
    else if (ch.typeDelta == 4) model.nodes[ch.idxNode].extensions["posewei"] = toVal(&val);
}

void gltfInterpolateLinear(tinygltf::Model& model, Channel& ch, int32 lowframe, int32 uppframe, float tt)
{
    Float4 low = ch.deltaKeyframes[lowframe].second;
    Float4 upp = ch.deltaKeyframes[uppframe].second;
    if (ch.typeDelta == 4)
    {
        //TBD weight
    }
    else if (ch.typeDelta == 1)//Translation
    {
        Float4 val = low * (1.0 - tt) + upp * tt;
        model.nodes[ch.idxNode].extensions["posetra"] = toVal(&val, sizeof(Float4));
    }
    else if (ch.typeDelta == 3)//Rotation
    {
        Quaternion lr = Quaternion(low.x, low.y, low.z, low.w);
        Quaternion ur = Quaternion(upp.x, upp.y, upp.z, upp.w);
        Quaternion mx = Math::Slerp(lr, ur, tt).normalize();
        Float4 val = Float4(mx.component.m128_f32[0], mx.component.m128_f32[1], mx.component.m128_f32[2], mx.component.m128_f32[3]);
        model.nodes[ch.idxNode].extensions["poserot"] = toVal(&val, sizeof(Float4));
    }
    else if (ch.typeDelta == 2)//Scale
    {
        Float4 val = low * (1.0 - tt) + upp * tt;
        model.nodes[ch.idxNode].extensions["posesca"] = toVal(&val, sizeof(Float4));
    }
}

//付録C：スプライン補間(https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#appendix-c-spline-interpolation)
template <typename T> T cubicSpline(float tt, T v0, T bb, T v1, T aa)
{
    const auto t2 = tt * tt;
    const auto t3 = t2 * tt;
    return (2 * t3 - 3 * t2 + 1) * v0 + (t3 - 2 * t2 + tt) * bb + (-2 * t3 + 3 * t2) * v1 + (t3 - t2) * aa;
}

void gltfInterpolateSpline(tinygltf::Model& model, Channel& ch, int32 lowframe, int32 uppframe, float lowtime, float upptime, float tt)
{
    auto delta = upptime - lowtime;
    auto v0 = ch.deltaKeyframes[3 * lowframe + 1].second;
    auto aa = delta * ch.deltaKeyframes[3 * uppframe + 0].second;
    auto bb = delta * ch.deltaKeyframes[3 * lowframe + 2].second;
    auto v1 = ch.deltaKeyframes[3 * uppframe + 1].second;

    if (ch.typeDelta == 4)
    {
        //TBD weight
    }
    else if (ch.typeDelta == 1) //Translate
    {
        Float4 val = cubicSpline(tt, v0, bb, v1, aa);
        model.nodes[ch.idxNode].extensions["posetra"] = toVal(&val, sizeof(Float4));
    }
    else if (ch.typeDelta == 3) //Rotation
    {
        Float4 val = cubicSpline(tt, v0, bb, v1, aa);
        Quaternion qt = Quaternion(val.x, val.y, val.z, val.w).normalize();
        val = Float4(qt.component.m128_f32[0], qt.component.m128_f32[1], qt.component.m128_f32[2], qt.component.m128_f32[3]);
        model.nodes[ch.idxNode].extensions["poserot"] = toVal(&val, sizeof(Float4));
    }
    else if (ch.typeDelta == 2) //Scale
    {
        Float4 val = cubicSpline(tt, v0, bb, v1, aa);
        model.nodes[ch.idxNode].extensions["posesca"] = toVal(&val, sizeof(Float4));
    }
}

void gltfCalcSkeleton(tinygltf::Model& model, tinygltf::Node& node, Mat4x4& matparent = Mat4x4().Identity())
{
    Array<float> r = toFloat(node.extensions["poserot"]);
    Array<float> t = toFloat(node.extensions["posetra"]);
    Array<float> s = toFloat(node.extensions["posesca"]);

    Quaternion rr = Quaternion(r[0], r[1], r[2], r[3]);
    Float3     tt = Float3(t[0], t[1], t[2]);
    Float3     ss = Float3(s[0], s[1], s[2]);

    Mat4x4 poserot = rr.toMatrix();
    Mat4x4 posetra = Mat4x4().Identity().Translate(tt);
    Mat4x4 posesca = Mat4x4().Identity().Scale(ss);
    Mat4x4 matpose = posesca * poserot * posetra;

    Mat4x4 matlocal = toMat(node.extensions["matlocal"]);

    if (0 == std::memcmp(&Mat4x4().Identity(), &matpose, sizeof(Mat4x4)))
        matpose = matlocal;

    Mat4x4 mat = matpose * matlocal.inverse();
    Mat4x4 matworld = mat * matlocal * matparent;

    node.extensions["matworld"] = toVal(&matworld, sizeof(Mat4x4));

    for (int32 cc = 0; cc < node.children.size(); cc++)
        gltfCalcSkeleton(model, model.nodes[node.children[cc]], matworld);
}

using Word4 = Vector4D<unsigned short>;

void gltfPrecomputeMesh(tinygltf::Model& model, int32 idxframe, AnimeModel& animodel, int32 aid, tinygltf::Node& node)
{
    PrecAnime& precanime = animodel.precAnimes[aid];
    for (int32 pp = 0; pp < model.meshes[node.mesh].primitives.size(); pp++)
    {
        auto& pr = model.meshes[node.mesh].primitives[pp];
        auto& map = model.accessors[pr.attributes["POSITION"]];

        int32 opos = 0, otex = 0, onormal = 0, ojoints = 0, oweights = 0, oidx = 0, stride = 0, texstride = 0;

        auto& bpos = *getBuffer(model, pr, "POSITION", &opos, &stride);
        auto& btex = *getBuffer(model, pr, "TEXCOORD_0", &otex, &texstride);
        auto& bnormal = *getBuffer(model, pr, "NORMAL", &onormal, &stride);
        auto& bjoint = *getBuffer(model, pr, "JOINTS_0", &ojoints, &stride);
        auto& bweight = *getBuffer(model, pr, "WEIGHTS_0", &oweights, &stride);
        auto& bidx = *getBuffer(model, pr, &oidx, &stride);

        Array<MeshVertex> vertices;
        for (int32 vv = 0; vv < map.count; vv++)
        {
            float* vertex = nullptr, * texcoord = nullptr, * normal = nullptr;
            vertex = (float*)&bpos.data.at(vv * 12 + opos);
            texcoord = (float*)&btex.data.at(vv * 8 + otex);
            normal = (float*)&bnormal.data.at(vv * 12 + onormal);

            MeshVertex mv;
            mv.position = Float3(vertex[0], vertex[1], vertex[2]);
            mv.texcoord = Float2(texcoord[0], texcoord[1]);
            mv.normal = Float3(normal[0], normal[1], normal[2]);

            //CPUスキニング
            if (pr.attributes["JOINTS_0"] && pr.attributes["WEIGHTS_0"])
            {
                Mat4x4 matskin = Mat4x4().Identity();

                auto joint = (uint16_t*)&bjoint.data.at(vv * 8 + ojoints);//1頂点あたり4JOINT
                auto weight = (float*)&bweight.data.at(vv * 16 + oweights);
                Word4  j4 = Word4(joint[0], joint[1], joint[2], joint[3]);
                Float4 w4 = Float4(weight[0], weight[1], weight[2], weight[3]);

                //スケルトン姿勢(接続されるボーンは4つ)
                matskin = w4.x * precanime.Joints[node.skin][j4.x] +
                    w4.y * precanime.Joints[node.skin][j4.y] +
                    w4.z * precanime.Joints[node.skin][j4.z] +
                    w4.w * precanime.Joints[node.skin][j4.w];

                if (pr.targets.size() > 0)
                {
                    precanime.Frames[idxframe].morphMatBuffer.emplace_back(matskin);
                    if (animodel.morphMesh.TexCoord.size() < animodel.morphMesh.TexCoordCount)
                        animodel.morphMesh.TexCoord.emplace_back(mv.texcoord);    //初回アニメの初回フレームで蓄積
                }

                auto matpos = matskin.transform(Float4(mv.position, 1.0f));
                auto matnor = matskin.inverse().transposed();
                mv.position = Float3(matpos.x, matpos.y, matpos.z) / matpos.w;
                mv.normal = matnor.transform(mv.normal);

            }
            vertices.emplace_back(mv);
        }

        MeshData md;
        if (pr.indices > 0)
        {
            auto& mapi = model.accessors[pr.indices];
            Array<uint32> indices;
            for (int32 ii = 0; ii < mapi.count; ii++)
            {
                uint32 idx;
                idx = (mapi.componentType == 5123) ? *(uint16*)&bidx.data.at(ii * 2 + oidx) :   //16bit
                    (mapi.componentType == 5125) ? *(uint32*)&bidx.data.at(ii * 4 + oidx) : 0;//32bit

                indices.emplace_back(idx);
            }

            //基準モデルのメッシュを生成
            md = MeshData(vertices, indices);

            vertices.clear();
            indices.clear();
        }

        auto texcol = 0;// テクスチャ頂点色識別子 b0=頂点色 b1=テクスチャ
        Texture tex;
        ColorF col;

        //Meshes->Primitive->Metarial->baseColorTexture.json_double_value.index->images
        if (pr.material >= 0)
        {
            auto& nt = model.materials[pr.material].additionalValues["normalTexture"];  //法線マップ
            int32 idx = -1;
            auto& mmv = model.materials[pr.material].values;
            auto& bcf = mmv["baseColorFactor"];                                         //色

            if (mmv.count("baseColorTexture"))
                idx = model.textures[(int)mmv["baseColorTexture"].json_double_value["index"]].source;

            //頂点色を登録
            if (bcf.number_array.size()) col = ColorF(bcf.number_array[0], bcf.number_array[1], bcf.number_array[2], bcf.number_array[3]);
            else                         col = ColorF(1, 1, 1, 1);

            //materials->textures->images
            if (idx >= 0 && model.images.size())
            {
                if (idxframe == 0)  //テクスチャと頂点色の登録はフレーム0に保存。
                {
                    tex = Texture();
                    if (model.images[idx].bufferView >= 0)
                    {
                        auto& bgfx = model.bufferViews[model.images[idx].bufferView];
                        auto bimg = &model.buffers[bgfx.buffer].data.at(bgfx.byteOffset);

                        ByteArray teximg((void*)bimg, bgfx.byteLength);
                        tex = Texture(std::move(teximg), TextureDesc::For3D);
                    }
                    else
                    {
                        auto& mii = model.images[idx].image;
                        ByteArray teximg((void*)&mii, mii.size());
                        tex = Texture(std::move(teximg), TextureDesc::For3D);
                    }
                }
                texcol |= 2;
            }
            texcol |= 1;
        }

        if (idxframe == 0)  //テクスチャと頂点色の登録はフレーム0に保存
        {
            precanime.meshTexs.emplace_back(tex);           // テクスチャ追加
            precanime.meshColors.emplace_back(col);         // 頂点色追加
        }

        precanime.Frames[idxframe].Meshes.emplace_back(DynamicMesh(md));    // メッシュ追加
        precanime.Frames[idxframe].TexCol.emplace_back(texcol);             // テクスチャ頂点色識別子追加
    }
}



// 1:口開小　2:口開大 3:眼閉 4:細目 5:下目 6ジト目 7:4:細目2 8:口笑小 9:口怒小 10:眉上 11眉困 12眉怒 13眉中 14眼閉2 
void gltfDrawSkinMesh(AnimeModel& animodel, Entity& ent,
    size_t istart = 0, size_t icount = -1)  //筆順表現用引数
{
    auto anime_no = ent.Maple->ArgI;    //アニメ番号取得
    auto& tra = ent.Trans;
    auto& sca = ent.Scale;
    auto& rot = ent.qRotate[0];

    PrecAnime& anime = animodel.precAnimes[(anime_no == -1) ? 0 : anime_no];
    double& cfr = anime.currentframe;
    auto& frame = anime.Frames[cfr];

    auto cntm = 0;                                  //モーフありメッシュの個数
    for (auto pp = 0; pp < frame.Meshes.size(); pp++)
    {
        auto& morphs = animodel.morphMesh.Targets[pp];

        if (morphs > 0)              //モーフありメッシュのみモーフ事前処理
        {
            Array<MeshVertex> morphmv = animodel.morphMesh.BasisBuffer[cntm]; //元メッシュのコピーを作業用で確保
            auto& buf = animodel.morphMesh.Buffers;

            for (auto vv = 0; vv < morphmv.size(); vv++)
            {
                Mat4x4& matskin = frame.morphMatBuffer[vv];
                for (auto ii = 0; ii < NUMM; ii++)   //3chモーフ合成(目、口、表情を非同期で動かす)
                {
                    auto& now = ent.Morph[ii].NowTarget;
                    auto& dst = ent.Morph[ii].DstTarget;
                    auto& idx = ent.Morph[ii].IndexTrans;
                    auto& wt = ent.Morph[ii].WeightTrans;

                    if (now == -1) continue;    //NowTargetが-1の場合はモーフ無効
                    if (idx == -1)             //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
                    {
                        morphmv[vv].position += buf[cntm * NUMM + ii][vv].position * (1 - wt[0]) + buf[cntm * NUMM + ii][vv].position * wt[0];
                        morphmv[vv].normal += buf[cntm * NUMM + ii][vv].normal * (1 - wt[0]) + buf[cntm * NUMM + ii][vv].normal * wt[0];
                    }
                    else
                    {
                        auto weight = (wt[idx] < 0) ? 0 : wt[idx];
                        morphmv[vv].position += buf[cntm * NUMM + now][vv].position * (1 - weight) + buf[cntm * NUMM + dst][vv].position * weight;
                        morphmv[vv].normal += buf[cntm * NUMM + now][vv].normal * (1 - weight) + buf[cntm * NUMM + dst][vv].normal * weight;
                    }
                }
                auto matpos = matskin.transform(Float4(morphmv[vv].position, 1.0f)); //モーフィング部分のみ動的スキニング
                auto matnor = matskin.inverse().transposed();
                morphmv[vv].position = Float3(matpos.x, matpos.y, matpos.z) / matpos.w;
                morphmv[vv].normal = matnor.transform(morphmv[vv].normal);
                morphmv[vv].texcoord = animodel.morphMesh.TexCoord[vv];
            }

            auto dm = frame.Meshes[pp];
            dm.fillVertices(morphmv);

            if (frame.TexCol[pp] & 2)      dm.rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, anime.meshTexs[pp]);
            else if (frame.TexCol[pp] & 1) dm.rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, anime.meshColors[pp]);
            else                           dm.rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount);

            cntm++;
        }

        else
        {
            if (frame.TexCol[pp] & 2)      frame.Meshes[pp].rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, anime.meshTexs[pp]);
            else if (frame.TexCol[pp] & 1) frame.Meshes[pp].rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount, anime.meshColors[pp]);
            else                           frame.Meshes[pp].rotated(rot).scaled(sca).translated(tra).drawSubset(istart, icount);
        }
    }
    cfr += ent.Maple->ArgF;
    if (floor(cfr) >= anime.Frames.size() - 2) cfr = 0;    //スプラインバグ隠蔽工作-2
    else if (floor(cfr) < 0) cfr = anime.Frames.size() - 2;
}

// 座標→文字列位置変換
int32 MaplePos2Idx(Float3 flt3, ChunkMap& chunkmap)
{
    Int3 cwhd = chunkmap.MapSize;
    Float3 cpos = chunkmap.Trans;
    int32 x = int32(flt3.x - cpos.x + 0.5);
    int32 y = int32(flt3.y - cpos.y + 0.5);
    int32 z = int32(flt3.z - cpos.z + 0.5);
    if (x < 0 || y < 0 || z < 0 || x >= cwhd.x || y >= cwhd.y || z >= cwhd.z) return 0;
    return x + z * cwhd.x + y * cwhd.x * cwhd.z;
}

int32 MaplePos2Idx(int32 x, int32 y, int32 z, int32 w, int32 h, int32 d)
{

    auto idx = x + z * w + y * w * h;
    if (idx < 0) idx = 0;
    if (idx >= w * h * d) idx = w * h * d - 1;
    return idx;
}

// chunkmap描画処理
void RenderMaple(MapleMap& maple, ChunkMap& chunkmap, ChunkMap& chunkatr)
{
    Array<Entity>& entities = maple.Entities;

    //物体マップクリア
    for (auto& ent : entities)
    {
        const String SPC(L"　");
        if (ent.Maple->Attr)//属性による影響あり
            chunkmap.Text[MaplePos2Idx(ent.Trans, chunkmap)] = SPC[0];
    }

    //マップ描画
    for (auto& ent : entities)
    {
        auto& cfg = ent.Maple;
        auto& animodel = cfg->AniModel;  //ANIモデルへの参照
        auto  noamodel = cfg->NoAModels; //NOAモデルのフレーム配列へのポインタ

        //こういのデフォルトで用意するのは良しとしても最後はユーザー処理をオーバーライドしたい。
        if (cfg->Mode == L"連" && noamodel.size() > 1) //モードが連番GLTF
        {
            auto frame = ((++ent.Frame) & 0x07C) >> 2;              //カウンタを1/4で連番を進める
            frame = (frame >= 16) ? 16 - (frame - 15) : frame;      //16フレームでループ

            noamodel[frame].Meshes.size();

            gltfDrawMesh(&noamodel[frame], ent, ent.qRotate[0], ent.Trans, ent.Scale,
                (size_t)Floor(ent.Start), (size_t)Floor(ent.Count));
        }

        // glTFフォント：描画先頭文字、描画終端文字、整数部は描画文字数、小数部は筆順表現、負は全字同時筆順
        else if (cfg->Mode.substr(0, 1) == L"線")
        {
            float radius = ent.Maple->ArgI;    //直線文字列で半径とは？
            float kerning = ent.Maple->ArgF;                                                           //文字列のカーニング量
            if (!ent.Maple->qSpin[0].isIdentity()) ent.qRotate[0] *= ent.Maple->qSpin[0]; //文字回転
            if (!ent.Maple->qSpin[1].isIdentity()) ent.qRotate[1] *= ent.Maple->qSpin[1]; //文字列回転

            gltfDrawString(&noamodel[0], ent.Maple->Mode, kerning, radius,
                ent.qRotate, ent.Trans, ent.Scale,
                (size_t)Floor(ent.Start), ent.Count);
        }

        //筆順表現の進度を格納する変数がないかも
        else if (cfg->Mode.substr(0, 1) == L"円")
        {
            float radius = ent.Maple->ArgI;
            float kerning = ent.Maple->ArgF;                                                              //文字列のカーニング量
            if (!ent.Maple->qSpin[0].isIdentity()) ent.qRotate[0] *= ent.Maple->qSpin[0]; //文字回転
            if (!ent.Maple->qSpin[1].isIdentity()) ent.qRotate[1] *= ent.Maple->qSpin[1]; //文字列回転

            gltfDrawString(&noamodel[0], ent.Maple->Mode, kerning, radius,
                ent.qRotate, ent.Trans, ent.Scale,
                (size_t)Floor(ent.Start), ent.Count);
        }

        else if (cfg->Mode == L"乱")    //乱番GLTF
        {
            int32 frame = 0;

            if (ent.Frame != -1) frame = ent.Frame; //乱番フレームはロード時に乱数でフレーム番号確定してる
            else frame = ent.Maple->ArgI;          //-1の場合は引数Iでプログラマブル
            gltfDrawMesh(&noamodel[frame], ent, ent.qRotate[0], ent.Trans, ent.Scale,
                (size_t)Floor(ent.Start), ent.Count);
        }

        //アニメNo＝-1は、すべてのアニメをデコードする。操作で切り替える場合に使用する。
        //アニメNo指定の場合は1つのみ。省メモリ
        else if (cfg->Mode == L"ア")    //アニメ付GLTF
        {
            gltfDrawSkinMesh(animodel, ent);
        }

        else                                        //その他ふつうの静的モデル
        {
            if (ent.Maple->Attr)                    //属性による影響あり
            {
                auto nowidx = MaplePos2Idx(ent.Trans, chunkmap);         //現在位置の属性を取ってくる
                auto name = chunkatr.Text.substr(nowidx, 1);
                for (auto& atr : maple.Attribs)
                {
                    if (atr.Name == name)                                         //取ってきた属性を選択
                    {
                        ent.Trans += atr.Trans;                                   //属性の重力ベクトルを加算
                        break;
                    }
                }
            }

            if (!ent.Maple->qSpin[0].isIdentity()) ent.qRotate[0] *= ent.Maple->qSpin[0];
            gltfDrawMesh(&noamodel[0], ent, ent.qRotate[0], ent.Trans, ent.Scale,
                (size_t)Floor(ent.Start), (size_t)Floor(ent.Count));
        }
    }

    //物体マップ再セット
    for (auto& ent : entities)
    {
        if (ent.Maple->Attr)    //属性による影響あり
            chunkmap.Text[MaplePos2Idx(ent.Trans, chunkmap)] = ent.Name[0];
    }

}

// chunkmap構築処理
void LoadMaple(MapleMap& maple, ChunkMap& chunkmap, ChunkMap& chunkatr, CSVReader& csv, String& maplepath)
{
    String state = L"!def";
    auto maxrow = csv.rows;

    Array< ChunkMap > map2ds;    // map2dsは/defで定義されるマップのリスト
    ChunkMap map;

    String token = L"";
    auto row = 0;

    //chunk検索
    auto& cm = chunkmap;
    for (; row <= maxrow; row++)
    {
        token = csv.get<String>(row, 0);

        if (token == L"/chunk")
        {
            cm.Scale = Float3(1, 1, 1);
            auto cols = csv.columns(row);
            for (auto col = 1; col < csv.columns(row); col++)
            {
                token = csv.get<String>(row, col).replace(L" ", L"");
                if (String::npos != token.indexOf(L"size("))
                {
                    String text = token.replace(L"size(", L"");
                    cm.MapSize.x = Parse<int32>(csv.get<String>(row, col + 0).replace(L"size(", L""));
                    cm.MapSize.y = Parse<int32>(csv.get<String>(row, col + 1));
                    cm.MapSize.z = Parse<int32>(csv.get<String>(row, col + 2));

                    //マップ格納域をL"　"でフィル
                    for (int32 ii = 0; ii < cm.MapSize.x * cm.MapSize.y * cm.MapSize.z; ii++) cm.Text += L"　";
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"trans("))
                {
                    cm.Trans.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"trans(", L""));
                    cm.Trans.y = Parse<float>(csv.get<String>(row, col + 1));
                    cm.Trans.z = Parse<float>(csv.get<String>(row, col + 2));
                    col += 2;
                    continue;
                }

//                if (String::npos != token.indexOf(L"scale("))
//                {
//                    cm.Scale.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"scale(", L""));
//                    cm.Scale.y = Parse<float>(csv.get<String>(row, col + 1));
//                    cm.Scale.z = Parse<float>(csv.get<String>(row, col + 2));
//                    col += 2;
//                    continue;
//                }

//                if (String::npos != token.indexOf(L"rotate("))
//                {
//                    cm.Rotate.x = Radians(Parse<float>(csv.get<String>(row, col + 0).replace(L"rotate(", L"")));
//                    cm.Rotate.y = Radians(Parse<float>(csv.get<String>(row, col + 1)));
//                    cm.Rotate.z = Radians(Parse<float>(csv.get<String>(row, col + 2)));
//                    col += 2;
//                    continue;
//                }
            }
            if (cm.MapSize.x && cm.MapSize.y && cm.MapSize.z)
            {
                row++;
                break;
            }
        }
    }
    chunkatr = chunkmap;    //属性マップも同様に初期化

    //entity/attrib登録
    for (; row <= maxrow; row++)
    {
        token = csv.get<String>(row, 0);
        if (token == L"/def") break;
        if (token == L"/end") row = maxrow;

        //entity設定
        if (token == L"/entity")
        {//                Type Name File  Trans   Scale   Rotate            Spin              Effect qRotate               qSpin               Mode ArgF ArgI
            MapleInfo mi = { L"" ,L"" , L"" ,{0,0,0},{1,1,1},{{0,0,0},{0,0,0}},{{0,0,0},{0,0,0}},0,0,{{0,0,0,0},{0,0,0,0}},{{0,0,0,0},{0,0,0,0}} ,L"",0, 0 };
            auto numframes = 1; //フレーム数は最低１

            for (auto col = 1; col <= csv.columns(row); col++)
            {
                token = csv.get<String>(row, col).replace(L" ", L"");
                if (String::npos != token.indexOf(L"file'"))
                {
                    String text = token.replace(L"file'", L"");
                    mi.Name = text.substr(0, 1);
                    mi.Filename = text.substr(2).replace(L"'", L"");
                    continue;
                }

                if (String::npos != token.indexOf(L"type'"))
                {
                    mi.Type = token.replace(L"type'", L"").replace(L"'", L"");
                    continue;
                }

                if (String::npos != token.indexOf(L"trans("))
                {
                    mi.Trans.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"trans(", L""));
                    mi.Trans.y = Parse<float>(csv.get<String>(row, col + 1));
                    mi.Trans.z = Parse<float>(csv.get<String>(row, col + 2));
                    mi.Trans.x *= cm.Scale.x;
                    mi.Trans.y *= cm.Scale.y;
                    mi.Trans.z *= cm.Scale.z;
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"scale("))
                {
                    mi.Scale.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"scale(", L""));
                    mi.Scale.y = Parse<float>(csv.get<String>(row, col + 1));
                    mi.Scale.z = Parse<float>(csv.get<String>(row, col + 2));
                    mi.Scale.x *= cm.Scale.x;
                    mi.Scale.y *= cm.Scale.y;
                    mi.Scale.z *= cm.Scale.z;
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"rotate("))
                {
                    // map.RotateMode = csv.get<String>(row, col + 0).replace(L"rotate(", L"");
                    mi.Rotate[0].x = Radians(Parse<float>(csv.get<String>(row, col + 0).replace(L"rotate(", L"")));
                    mi.Rotate[0].y = Radians(Parse<float>(csv.get<String>(row, col + 1)));
                    mi.Rotate[0].z = Radians(Parse<float>(csv.get<String>(row, col + 2)));
                    auto& mr = mi.Rotate[0];
                    mi.qRotate[0] = Quaternion::RollPitchYaw(mr.x, mr.y, mr.z);
                    col += 2;

                    if (String::npos == csv.get<String>(row, col).indexOf(L")"))   //値6個の場合(文字列)は追加で取得
                    {
                        mi.Rotate[1].x = Radians(Parse<float>(csv.get<String>(row, col + 1)));
                        mi.Rotate[1].y = Radians(Parse<float>(csv.get<String>(row, col + 2)));
                        mi.Rotate[1].z = Radians(Parse<float>(csv.get<String>(row, col + 3)));
                        auto& mr1 = mi.Rotate[1];
                        mi.qRotate[1] = Quaternion::RollPitchYaw(mr1.x, mr1.y, mr1.z);
                        col += 3;
                    }
                    continue;
                }

                if (String::npos != token.indexOf(L"spin("))
                {
                    mi.Spin[0].x = Radians(Parse<float>(csv.get<String>(row, col + 0).replace(L"spin(", L"")));
                    mi.Spin[0].y = Radians(Parse<float>(csv.get<String>(row, col + 1)));
                    mi.Spin[0].z = Radians(Parse<float>(csv.get<String>(row, col + 2)));
                    auto& ms0 = mi.Spin[0];
                    mi.qSpin[0] = Quaternion::RollPitchYaw(ms0.x, ms0.y, ms0.z);
                    col += 2;

                    if (String::npos == csv.get<String>(row, col).indexOf(L")"))   //値6個の場合は追加で取得
                    {
                        mi.Spin[1].x = Radians(Parse<float>(csv.get<String>(row, col + 1)));
                        mi.Spin[1].y = Radians(Parse<float>(csv.get<String>(row, col + 2)));
                        mi.Spin[1].z = Radians(Parse<float>(csv.get<String>(row, col + 3)));
                        auto& ms1 = mi.Spin[1];
                        mi.qSpin[1] = Quaternion::RollPitchYaw(ms1.x, ms1.y, ms1.z);
                        col += 3;
                    }
                    continue;
                }

                if (String::npos != token.indexOf(L"attr("))
                {
                    mi.Attr = Parse<uint32>(csv.get<String>(row, col + 0).replace(L"attr(", L""));
                    mi.Dummy = Parse<float>(csv.get<String>(row, col + 1));
                    col += 1;
                    continue;
                }

                if (String::npos != token.indexOf(L"mode(")) //連：連番GLTF数、乱：ランダム数　ア：アニメーションGLTF
                {
                    mi.Mode = csv.get<String>(row, col++).replace(L"mode(", L"").replace(L"'", L"");//シングルクォート使えない

                    if (mi.Type == L"字")
                    {
                        mi.ArgI = Parse<uint32>(csv.get<String>(row, col++));  //半径
                        mi.ArgF = Parse<float>(csv.get<String>(row, col++));   //カーニング値
                    }
                    else if (mi.Mode == L"ア")
                    {
                        mi.ArgI = Parse<uint32>(csv.get<String>(row, col++));  //アニメの場合はアニメ番号、再生速度[10%=0.1]
                        mi.ArgF = Parse<float>(csv.get<String>(row, col++));
                    }

                    else numframes = Parse<uint32>(csv.get<String>(row, col++));  //その他はフレーム数
                    continue;
                }
            }

            //glTFモデル取得
            std::string err, warn;
            tinygltf::TinyGLTF loader;
            tinygltf::Model model;

            if (numframes > 1)  // 連番(乱番)glTFの場合 
            {
                String name = mi.Filename;
                auto pre = name.substr(0, mi.Filename.indexOf(L"(") + 1);
                auto post = name.substr(mi.Filename.indexOf(L")"));

                for (auto ii = 0; ii < numframes; ii++)
                {
                    auto filename = pre + Format(ii) + post;
                    bool result = loader.LoadBinaryFromFile(&model, &err, &warn, (maplepath + filename).narrow());
                    NoAModel noamodel;
                    gltfSetupModel(noamodel, model);
                    mi.NoAModels.emplace_back(noamodel);
                }
            }
            else                        // 単一glTF(numframe=1)
            {
                if (mi.Mode == L"ア")  //アニメ対応GLTF
                {
                    bool result = loader.LoadBinaryFromFile(&model, &err, &warn, (maplepath + mi.Filename).narrow());
                    if( result ) result = loader.LoadASCIIFromFile(&model, &err, &warn, (maplepath + mi.Filename).narrow());
//                    gltfSetupModel( mi.AniModel, model, mi.ArgI);
                    maple.Maples.emplace_back(mi);
                    gltfSetupModel( maple.Maples.back(), model, mi.ArgI);
                }
                else                    //アニメなしGLTF
                {
                    bool result = loader.LoadBinaryFromFile(&model, &err, &warn, (maplepath + mi.Filename).narrow());
                    if( result ) result = loader.LoadASCIIFromFile(&model, &err, &warn, (maplepath + mi.Filename).narrow());
                    NoAModel noamodel;
                    gltfSetupModel(noamodel, model);
                    mi.NoAModels.emplace_back(noamodel);
                    maple.Maples.emplace_back(mi);
                }
            }
//            maple.Maples.emplace_back(mi);
        }

        //attr設定
        if (token == L"/attr")
        {//                  Type Name  Trans Scale   Rotate  qRotate  Mode ArgF ArgI Tag
            Attribute ai = { L"" ,L"" , {0,0,0},{1,1,1},{0,0,0},{0,0,0,0} ,L"",0, 0     ,L"" };
            auto numframes = 1; //フレーム数は最低１

            for (auto col = 1; col <= csv.columns(row); col++)
            {
                token = csv.get<String>(row, col).replace(L" ", L"");
                if (String::npos != token.indexOf(L"name'"))
                {
                    String text = token.replace(L"name'", L"");
                    ai.Name = text.substr(0, 1);
                    continue;
                }

                if (String::npos != token.indexOf(L"type'"))
                {
                    ai.Type = token.replace(L"type'", L"").replace(L"'", L"");
                    continue;
                }

                if (String::npos != token.indexOf(L"trans("))
                {
                    ai.Trans.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"trans(", L""));
                    ai.Trans.y = Parse<float>(csv.get<String>(row, col + 1));
                    ai.Trans.z = Parse<float>(csv.get<String>(row, col + 2));
                    ai.Trans.x *= cm.Scale.x;
                    ai.Trans.y *= cm.Scale.y;
                    ai.Trans.z *= cm.Scale.z;
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"scale("))
                {
                    ai.Scale.x = Parse<float>(csv.get<String>(row, col + 0).replace(L"scale(", L""));
                    ai.Scale.y = Parse<float>(csv.get<String>(row, col + 1));
                    ai.Scale.z = Parse<float>(csv.get<String>(row, col + 2));
                    ai.Scale.x *= cm.Scale.x;
                    ai.Scale.y *= cm.Scale.y;
                    ai.Scale.z *= cm.Scale.z;
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"rotate("))
                {
                    // map.RotateMode = csv.get<String>(row, col + 0).replace(L"rotate(", L"");
                    ai.Rotate.x = Radians(Parse<float>(csv.get<String>(row, col + 0).replace(L"rotate(", L"")));
                    ai.Rotate.y = Radians(Parse<float>(csv.get<String>(row, col + 1)));
                    ai.Rotate.z = Radians(Parse<float>(csv.get<String>(row, col + 2)));
                    auto& mr = ai.Rotate;
                    ai.qRotate = Quaternion::RollPitchYaw(mr.x, mr.y, mr.z);
                    col += 2;
                    continue;
                }

                if (String::npos != token.indexOf(L"mode(")) //空：
                {
                    ai.Mode = csv.get<String>(row, col++).replace(L"mode(", L"").replace(L"'", L"");//シングルクォート使えない

                    if (ai.Type == L"空")
                    {
                        ai.ArgI = Parse<uint32>(csv.get<String>(row, col++));  //半径
                        ai.ArgF = Parse<float>(csv.get<String>(row, col++));   //カーニング値
                    }
                    else numframes = Parse<uint32>(csv.get<String>(row, col++));  //その他はフレーム数

                    continue;
                }

                if (String::npos != token.indexOf(L"tag'")) //空：
                {
                    ai.Tag = csv.get<String>(row, col++).replace(L"tag'", L"").replace(L"'", L"");//シングルクォート使えない
                    continue;
                }
            }

            maple.Attribs.emplace_back(ai);
        }

    }

    while (row <= maxrow)
    {
        //def検索
        for (; row <= maxrow; row++)
        {
            token = csv.get<String>(row, 0);
            if (token == L"/def") break;
            if (token == L"/end") row = maxrow;
        }
        row++;

        //def取得
        for (; row <= maxrow; row++)
        {
            for (auto col = 0; col <= csv.columns(row); col++)
            {
                token = csv.get<String>(row, col).replace(L" ", L"");

                if (token == L"/map" || token == L"/atr") break;
                if (token == L"/xz" || token == L"/xy" || token == L"/yz")
                {
                    map = { 0, token, L"", Int3(0, 0, 0), Int3(0, 0, 0) };
                    for (auto offset = 0; offset < 8; offset++)
                    {
                        String str = csv.get<String>(row, col + offset);
                        if (String::npos != str.indexOf(L"name'"))
                        {
                            map.Name = str.replace(L"name'", L"").replace(L"'", L"");
                        }

                        else if (String::npos != str.indexOf(L"layer("))    //レイヤー番号:0は省略可
                        {
                            map.Layer = Parse<uint32>(csv.get<String>(row, col + offset + 0).replace(L"layer(", L""));
                        }

                        else if (String::npos != str.indexOf(L"pos("))
                        {
                            map.MapPos.x = Parse<int32>(csv.get<String>(row, col + offset + 0).replace(L"pos(", L""));
                            map.MapPos.y = Parse<int32>(csv.get<String>(row, col + offset + 1));
                            map.MapPos.z = Parse<int32>(csv.get<String>(row, col + offset + 2));
                            offset += 2;
                        }

                        else if (String::npos != str.indexOf(L"size("))
                        {
                            map.MapSize.x = Parse<int32>(csv.get<String>(row, col + offset + 0).replace(L"size(", L""));
                            map.MapSize.y = Parse<int32>(csv.get<String>(row, col + offset + 1));
                            map.MapSize.z = Parse<int32>(csv.get<String>(row, col + offset + 2));
                            offset += 2;
                        }
                    }
                    map2ds.emplace_back(map);
                    col += 7;
                }
            }
            if (token == L"/map" || token == L"/atr") break;
        }

        //map/atr取得
        row++;

        if (token == L"/map")
        {
            auto mapnum = map2ds.size();
            for (int32 mc = 0; mc < mapnum; mc++)       //スクリプトからマップ記号取得
            {
                auto& mp = map2ds[mc];
                auto& chunkwidth = chunkmap.MapSize.x;
                auto& chunkheight = chunkmap.MapSize.y;
                auto& chunkdepth = chunkmap.MapSize.z;
                String srctext = L"";

                for (int32 mr = 0; mr < mp.MapSize.x; mr++)             // 全行ループ
                {
                    token = csv.get<String>(row + mr, mc);              // マップ文字列取得
                    if (token.length < chunkwidth)                      // 指定サイズより小さい場合はパッド
                        for (auto i = token.length; i < mp.MapSize.x; i++)
                            token += L"　";

                    else token = token.substr(0, mp.MapSize.x);         // 指定サイズより大きい場合は切り詰め
                    srctext += token;                                   // 1行追加
                    // LOG(srctext.substr(mr * mp.Size.x, mp.Size.x));
                }

                //maps→chunkmaps
                auto& cm = chunkmap;
                Int3 org = mp.MapPos;                                       // マップ文字列の原点取得
                if (mp.Type == L"/xz")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)              // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkdepth) continue;           // Z範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)          // 原点+Yオフセット
                        {
                            if ((org.y + my) >= chunkheight) continue;      // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)      // 原点+Xオフセット
                            {
                                if ((org.x + mx) >= chunkwidth) continue;   // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;          // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz) * chunkwidth * chunkheight;
                                dst += (chunkheight - 1 - (org.y + my)) * chunkwidth;
                                dst += (org.x + mx);                        // 原点考慮して3次元文字列の位置を算出
                                cm.Text = cm.Text.substr(0, dst) + srctext.substr(src, 1) + cm.Text.substr(dst + 1); // 特定位置の１文字を置き換え
                            }
                            //                        if (mp.Name == L"id3")
                            //LOG(cm.Text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }
                else if (mp.Type == L"/xy")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)                  // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkheight) continue;             // Y範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)              // 原点+Yオフセット
                        {
                            if (0 > (org.y - my) || (org.y - my) >= chunkdepth) continue;          // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)          // 原点+Xオフセット
                            {
                                if ((org.x + mx) >= chunkwidth) continue;      // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;              // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz) * chunkwidth;
                                dst += (chunkheight - 1 - (org.y - my)) * chunkwidth * chunkheight;
                                dst += (org.x + mx);                        // 原点考慮した差替文字の位置を算出

                                cm.Text = cm.Text.substr(0, dst) + srctext.substr(src, 1) + cm.Text.substr(dst + 1); // 特定位置の文字を置き換え
                            }
                            // LOG(cvm.text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }
                else if (mp.Type == L"/yz")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)                  // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkwidth) continue;             // Z範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)              // 原点+Yオフセット
                        {
                            if (0 > (org.y - my) || (org.y - my) >= chunkheight) continue;          // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)          // 原点+Xオフセット
                            {
                                if (0 > (org.x - mx) || (org.x - mx) >= chunkdepth) continue;      // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;              // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz);
                                dst += (chunkheight - 1 - (org.y - my)) * chunkwidth * chunkheight;
                                dst += (org.x - mx) * chunkwidth;          // 原点考慮した差替文字の位置を算出

                                cm.Text = cm.Text.substr(0, dst) + srctext.substr(src, 1) + cm.Text.substr(dst + 1); // 特定位置の文字を置き換え
                            }
                            // LOG(cvm.text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }

                if (mc == (mapnum - 1))  // /mapブロック完了
                {
                    for (auto cy = 0; cy < chunkdepth; cy++)
                    {
                        for (auto cz = 0; cz < chunkheight; cz++)
                        {
                            for (auto cx = 0; cx < chunkwidth; cx++)
                            {
                                // マップから記号を取得
                                String tile = cm.Text.substr(MaplePos2Idx(cx, cy, cz, chunkwidth, chunkheight, chunkdepth), 1);
                                Float3 trans(cm.Trans.x + cx * cm.Scale.x,
                                    cm.Trans.y + cy * cm.Scale.y,
                                    cm.Trans.z + cz * cm.Scale.z);    // マップ定義(x,y)→xz平面(x,z,y)変換

                                Entity entity;
                                for (int32 ii = 0; ii < maple.Maples.size(); ii++)
                                {
                                    if (maple.Maples[ii].Name == tile)
                                    {
                                        MapleInfo* mi = &maple.Maples[ii];

                                        int32 cnt = 0;
                                        for (auto ent : maple.Entities)    //既存の登録数をカウント
                                        {
                                            if (ent.Name == tile && ent.Trans == mi->Trans + trans)
                                            {
                                                cnt++;
                                                break;
                                            }
                                        }
                                        if (cnt) break; //登録あればキャンセル

                                        uint32 frameno = 0;

                                        float rr = mi->Rotate[0].x;
                                        float rp = mi->Rotate[0].y;
                                        float ry = mi->Rotate[0].z;

                                        entity = {
                                            mi->Type, mi->Name, frameno, mi,
                                            mi->Trans + trans , mi->Scale ,
                                            { Float3(rr,rp,ry), Float3(0,0,0) },
                                            { mi->qRotate[0] , mi->qRotate[1] },
                                            0,
                                            -1,
                                            { 1,0,0,0,-1,{0,-1}, 1,0,0,0,-1,{0,-1}, 1,0,0,0,-1,{0,-1} },  // MorphInfo
                                            {0}, //CtrlI※[0]はArgIの履歴管理用
                                            {0}  //CtrlF
                                        };

                                        if (mi->Mode == L"乱") entity.Frame = Random() * mi->NoAModels.size();    //乱数モードの場合はフレーム番号を適当に決める

                                        maple.Entities.emplace_back(entity);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    row += mp.MapSize.y;
                    map2ds.clear();
                }
            }
        }

        else if (token == L"/atr")
        {
            auto atrnum = map2ds.size();
            for (int32 mc = 0; mc < atrnum; mc++)       //スクリプトからマップ記号取得
            {
                auto& mp = map2ds[mc];
                auto& chunkwidth = chunkmap.MapSize.x;
                auto& chunkheight = chunkmap.MapSize.y;
                auto& chunkdepth = chunkmap.MapSize.z;
                String srctext = L"";

                for (int32 mr = 0; mr < mp.MapSize.x; mr++)         // 全行ループ
                {
                    token = csv.get<String>(row + mr, mc);          // マップ文字列取得
                    if (token.length < chunkwidth)                  // 指定サイズより小さい場合はパッド
                        for (auto i = token.length; i < mp.MapSize.x; i++)
                            token += L"　";

                    else token = token.substr(0, mp.MapSize.x);      // 指定サイズより大きい場合は切り詰め
                    srctext += token;                                // 1行追加
                    LOG(token);
                }

                //maps→chunkatrs
                auto& ca = chunkatr;
                Int3 org = mp.MapPos;                             // マップ文字列の原点取得
                if (mp.Type == L"/xz")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)               // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkdepth) continue;            // Z範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)           // 原点+Yオフセット
                        {
                            if ((org.y + my) >= chunkheight) continue;       // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)       // 原点+Xオフセット
                            {
                                if ((org.x + mx) >= chunkwidth) continue;    // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;              // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz) * chunkwidth * chunkheight;
                                dst += (chunkheight - 1 - (org.y + my)) * chunkwidth;
                                dst += (org.x + mx);                        // 原点考慮して3次元文字列の位置を算出
                                ca.Text = ca.Text.substr(0, dst) + srctext.substr(src, 1) + ca.Text.substr(dst + 1); // 特定位置の１文字を置き換え
                            }
                            //                        if (mp.Name == L"id3")
                            //LOG(cm.Text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }
                else if (mp.Type == L"/xy")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)                  // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkheight) continue;             // Y範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)              // 原点+Yオフセット
                        {
                            if (0 > (org.y - my) || (org.y - my) >= chunkdepth) continue;          // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)          // 原点+Xオフセット
                            {
                                if ((org.x + mx) >= chunkwidth) continue;      // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;              // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz) * chunkwidth;
                                dst += (chunkheight - 1 - (org.y - my)) * chunkwidth * chunkheight;
                                dst += (org.x + mx);                        // 原点考慮した差替文字の位置を算出

                                ca.Text = ca.Text.substr(0, dst) + srctext.substr(src, 1) + ca.Text.substr(dst + 1); // 特定位置の文字を置き換え
                            }
                            // LOG(cvm.text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }
                else if (mp.Type == L"/yz")
                {
                    for (auto mz = 0; mz < mp.MapSize.z; mz++)                  // 原点+Zオフセット
                    {
                        if ((org.z + mz) >= chunkwidth) continue;               // Z範囲外処理キャンセル
                        for (auto my = 0; my < mp.MapSize.y; my++)              // 原点+Yオフセット
                        {
                            if (0 > (org.y - my) || (org.y - my) >= chunkheight) continue;          // Y範囲外処理キャンセル
                            for (auto mx = 0; mx < mp.MapSize.x; mx++)          // 原点+Xオフセット
                            {
                                if (0 > (org.x - mx) || (org.x - mx) >= chunkdepth) continue;      // X範囲外処理キャンセル

                                auto src = my * mp.MapSize.x + mx;              // 原点考慮した差替文字の位置を算出

                                auto dst = 0;
                                dst += (org.z + mz);
                                dst += (chunkheight - 1 - (org.y - my)) * chunkwidth * chunkheight;
                                dst += (org.x - mx) * chunkwidth;                // 原点考慮した差替文字の位置を算出

                                ca.Text = ca.Text.substr(0, dst) + srctext.substr(src, 1) + ca.Text.substr(dst + 1); // 特定位置の文字を置き換え
                            }
                            // LOG(cvm.text.substr(mz * chunkwidth * chunkheight + my * chunkwidth, chunkwidth));
                        }
                    }
                }

                if (mc == (atrnum - 1))  // /atrブロック完了
                {
                    row += mp.MapSize.y;
                    map2ds.clear();
                }
            }
        }
    }
}
