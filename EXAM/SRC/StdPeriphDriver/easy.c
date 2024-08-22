#include "./inc/easy.h"

void Print_Memory(uint8_t *p, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		PRINT("%02x ",*(p+i));
	}
}
void DoPRINT(const char* file, int line, const char* date, const char* time, const char* func,  const char* format, ...)
{
    static char buffer[256];

    int offset = snprintf(buffer, sizeof(buffer), "[%s:%d] ", func, line);
    va_list args;
    va_start(args, format);
    vsnprintf(buffer + offset, sizeof(buffer) - offset, format, args);
    va_end(args);

	PRINT("%s", buffer);
}

const char* BLE_Opcode2str(uint8_t m) 
{
	if(m == ATT_ERROR_RSP) return "ATT_ERROR_RSP";
	if(m == ATT_EXCHANGE_MTU_REQ) return "ATT_EXCHANGE_MTU_REQ";
	if(m == ATT_EXCHANGE_MTU_RSP) return "ATT_EXCHANGE_MTU_RSP";
	if(m == ATT_FIND_INFO_REQ) return "ATT_FIND_INFO_REQ";
	if(m == ATT_FIND_INFO_RSP) return "ATT_FIND_INFO_RSP";
	if(m == ATT_FIND_BY_TYPE_VALUE_REQ) return "ATT_FIND_BY_TYPE_VALUE_REQ";
	if(m == ATT_FIND_BY_TYPE_VALUE_RSP) return "ATT_FIND_BY_TYPE_VALUE_RSP";
	if(m == ATT_READ_BY_TYPE_REQ) return "ATT_READ_BY_TYPE_REQ";
	if(m == ATT_READ_BY_TYPE_RSP) return "ATT_READ_BY_TYPE_RSP";
	if(m == ATT_READ_REQ) return "ATT_READ_REQ";
	if(m == ATT_READ_RSP) return "ATT_READ_RSP";
	if(m == ATT_READ_BLOB_REQ) return "ATT_READ_BLOB_REQ";
	if(m == ATT_READ_BLOB_RSP) return "ATT_READ_BLOB_RSP";
	if(m == ATT_READ_MULTI_REQ) return "ATT_READ_MULTI_REQ";
	if(m == ATT_READ_MULTI_RSP) return "ATT_READ_MULTI_RSP";
	if(m == ATT_READ_BY_GRP_TYPE_REQ) return "ATT_READ_BY_GRP_TYPE_REQ";
	if(m == ATT_READ_BY_GRP_TYPE_RSP) return "ATT_READ_BY_GRP_TYPE_RSP";
	if(m == ATT_WRITE_REQ) return "ATT_WRITE_REQ";
	if(m == ATT_WRITE_RSP) return "ATT_WRITE_RSP";
	if(m == ATT_PREPARE_WRITE_REQ) return "ATT_PREPARE_WRITE_REQ";
	if(m == ATT_PREPARE_WRITE_RSP) return "ATT_PREPARE_WRITE_RSP";
	if(m == ATT_EXECUTE_WRITE_REQ) return "ATT_EXECUTE_WRITE_REQ";
	if(m == ATT_EXECUTE_WRITE_RSP) return "ATT_EXECUTE_WRITE_RSP";
	if(m == ATT_HANDLE_VALUE_NOTI) return "ATT_HANDLE_VALUE_NOTI";
	if(m == ATT_HANDLE_VALUE_IND) return "ATT_HANDLE_VALUE_IND";
	if(m == ATT_HANDLE_VALUE_CFM) return "ATT_HANDLE_VALUE_CFM";
	return "unknown ATT msg";
}

/**
 * 所有的UUID都唯一
 * 1. GATT Service（服务） 16 位
 * 2. GATT Characteristic（特征） 服务的基本元素，它包含数据值和与该值相关的属性。特征可以是可读的、可写的或通知的。 16 位或 128 位。
 * 3. GATT Descriptors（描述符） 提供特征的附加信息或属性，通常用于定义如何处理特征数据。例如，描述符可以用于说明一个特征的配置或属性。  16 位或 128 位。
 *
 * 服务（Service）包含一个或多个特征（Characteristic）。
 * 每个特征可以有一个或多个描述符（Descriptor）来提供更多信息。
 * 
 * 例如
 * 服务：心率服务（UUID: 0x180D）
 *   特征：心率测量特征（UUID: 0x2A37）
 *     描述符：用户描述符（UUID: 0x2901）用于说明心率测量的内容。
 * 
 * http://www.atmcu.com/2280.html
*/
const char* BLE_UUID2str(uint16_t m)
{
	// GATT 声明
	if(m == 0x2800) return "UUID Declarations 主要服务";
	if(m == 0x2801) return "UUID Declarations 次要服务";
	if(m == 0x2802) return "UUID Declarations 包括";
	if(m == 0x2803) return "UUID Declarations 特征声明";

	// Service
	// GSS（Generic Access Profile Service）：用于设备的通用访问配置。
	if(m == 0x1800) return "UUID Service 通用访问";
	if(m == 0x1801) return "UUID Service 通用属性";
	if(m == 0x1802) return "UUID Service 即时闹钟";
	if(m == 0x1803) return "UUID Service 连接丢失";
	if(m == 0x1804) return "UUID Service 发送功率";
	if(m == 0x1805) return "UUID Service 当前时间";
	if(m == 0x1806) return "UUID Service 参照时间更新";
	if(m == 0x1807) return "UUID Service 夏令时更改";
	if(m == 0x1808) return "UUID Service 葡萄糖";
	if(m == 0x1809) return "UUID Service 温度计";
	if(m == 0x180A) return "UUID Service 设备信息";
	if(m == 0x180D) return "UUID Service 心率";
	if(m == 0x180E) return "UUID Service 手机报警状态";
	if(m == 0x180F) return "UUID Service 电池数据";
	if(m == 0x1810) return "UUID Service 血压";
	if(m == 0x1811) return "UUID Service 闹钟通知";
	if(m == 0x1812) return "UUID Service HID设备";
	if(m == 0x1813) return "UUID Service 扫描参数";
	if(m == 0x1814) return "UUID Service 跑步速度、节奏";
	if(m == 0x1815) return "UUID Service 自动化输入输出";
	if(m == 0x1816) return "UUID Service 循环速度、节奏";
	if(m == 0x1818) return "UUID Service 骑行能量";
	if(m == 0x1819) return "UUID Service 定位及导航";
	if(m == 0x181A) return "UUID Service 环境传感";
	if(m == 0x181B) return "UUID Service 身体组成";
	if(m == 0x181C) return "UUID Service 用户数据";
	if(m == 0x181D) return "UUID Service 体重秤";
	if(m == 0x181E) return "UUID Service 设备绑定管理";
	if(m == 0x181F) return "UUID Service 动态血糖检测";
	if(m == 0x1820) return "UUID Service 互联网协议支持";
	if(m == 0x1821) return "UUID Service 室内定位";
	if(m == 0x1822) return "UUID Service 脉搏血氧计";
	if(m == 0x1823) return "UUID Service HTTP代理";
	if(m == 0x1824) return "UUID Service 传输发现";
	if(m == 0x1825) return "UUID Service 对象传输";
	if(m == 0x1826) return "UUID Service 健康设备";
	if(m == 0x1827) return "UUID Service 节点配置";
	if(m == 0x1828) return "UUID Service 节点代理";
	if(m == 0x1829) return "UUID Service 重连配置";
	if(m == 0x183A) return "UUID Service 胰岛素给药";
	// BSS（Battery Service）：用于电池相关信息的服务，例如电量水平。
	if(m == 0x183B) return "UUID Service 二元传感器";
	// EMCS（Environmental Monitoring Service）：用于环境监测的服务，例如温度、湿度等数据的传输。
	if(m == 0x183C) return "UUID Service 应急配置";

	// Characteristic
	// GSS（Generic Access Profile Service）：用于设备的通用访问配置。
	if(m == 0x2A00) return "UUID Characteristic 设备名称";
	if(m == 0x2A01) return "UUID Characteristic 外观";
	if(m == 0x2A02) return "UUID Characteristic 周边隐私标志";
	if(m == 0x2A03) return "UUID Characteristic 重新连接地址";
	if(m == 0x2A04) return "UUID Characteristic 外围设备首选连接参数";
	if(m == 0x2A05) return "UUID Characteristic 服务已更改";
	if(m == 0x2A06) return "UUID Characteristic 报警等级";
	if(m == 0x2A07) return "UUID Characteristic 发射功率等级";
	if(m == 0x2A08) return "UUID Characteristic 日期时间";
	if(m == 0x2A09) return "UUID Characteristic 星期几";
	if(m == 0x2A0A) return "UUID Characteristic 日期时间天";
	if(m == 0x2A0B) return "UUID Characteristic 具体时间100";
	if(m == 0x2A0C) return "UUID Characteristic 具体时间256";
	if(m == 0x2A0D) return "UUID Characteristic 夏令时偏移";
	if(m == 0x2A0E) return "UUID Characteristic 时区";
	if(m == 0x2A0F) return "UUID Characteristic 当地时间信息";
	if(m == 0x2A10) return "UUID Characteristic 次要时区";
	if(m == 0x2A11) return "UUID Characteristic 夏令时";
	if(m == 0x2A12) return "UUID Characteristic 时间精度";
	if(m == 0x2A13) return "UUID Characteristic 时间来源";
	if(m == 0x2A14) return "UUID Characteristic 参考时间信息";
	if(m == 0x2A15) return "UUID Characteristic 时间广播";
	if(m == 0x2A16) return "UUID Characteristic 时间更新控制点";
	if(m == 0x2A17) return "UUID Characteristic 时间更新状态";
	if(m == 0x2A18) return "UUID Characteristic 血糖测量";
	if(m == 0x2A19) return "UUID Characteristic 电池电量";
	if(m == 0x2A1A) return "UUID Characteristic 电池电量状态";
	if(m == 0x2A1B) return "UUID Characteristic 电池电量等级";
	if(m == 0x2A1C) return "UUID Characteristic 温度测量";
	if(m == 0x2A1D) return "UUID Characteristic 温度类型";
	if(m == 0x2A1E) return "UUID Characteristic 中间的温度";
	if(m == 0x2A1F) return "UUID Characteristic 温度摄氏";
	if(m == 0x2A20) return "UUID Characteristic 温度华氏度";
	if(m == 0x2A21) return "UUID Characteristic 测量间隔";
	if(m == 0x2A22) return "UUID Characteristic 启动键盘输入报告";
	if(m == 0x2A23) return "UUID Characteristic 系统编号";
	if(m == 0x2A24) return "UUID Characteristic 型号字符";
	if(m == 0x2A25) return "UUID Characteristic 序列号字符";
	if(m == 0x2A26) return "UUID Characteristic 固件修订字符";
	if(m == 0x2A27) return "UUID Characteristic 硬件修订字符";
	if(m == 0x2A28) return "UUID Characteristic 软件修订版字符";
	if(m == 0x2A29) return "UUID Characteristic 制造商名称字符";
	if(m == 0x2A2A) return "UUID Characteristic 11073-20601法规认证数据列表";
	if(m == 0x2A2B) return "UUID Characteristic 当前时间";
	if(m == 0x2A2C) return "UUID Characteristic 磁偏角";
	if(m == 0x2A2F) return "UUID Characteristic 位置2D";
	if(m == 0x2A30) return "UUID Characteristic 位置3D";
	if(m == 0x2A31) return "UUID Characteristic 扫描刷新";
	if(m == 0x2A32) return "UUID Characteristic 启动键盘输出报告";
	if(m == 0x2A33) return "UUID Characteristic 启动鼠标输入报告";
	if(m == 0x2A34) return "UUID Characteristic 葡萄糖测量环境";
	if(m == 0x2A35) return "UUID Characteristic 血压测量";
	if(m == 0x2A36) return "UUID Characteristic 中间的气囊压力";
	if(m == 0x2A37) return "UUID Characteristic 心率测量";
	if(m == 0x2A38) return "UUID Characteristic 人体感应器位置";
	if(m == 0x2A39) return "UUID Characteristic 心率控制点";
	if(m == 0x2A3A) return "UUID Characteristic 可移动的";
	if(m == 0x2A3B) return "UUID Characteristic 所需服务";
	if(m == 0x2A3C) return "UUID Characteristic 科学温度（摄氏度）";
	if(m == 0x2A3D) return "UUID Characteristic 字符串";
	if(m == 0x2A3E) return "UUID Characteristic 网络可用性";
	if(m == 0x2A3F) return "UUID Characteristic 报警状态";
	if(m == 0x2A40) return "UUID Characteristic 铃声控制点";
	if(m == 0x2A41) return "UUID Characteristic 铃声设置";
	if(m == 0x2A42) return "UUID Characteristic 报警类别ID位掩码";
	if(m == 0x2A43) return "UUID Characteristic 报警类别ID";
	if(m == 0x2A44) return "UUID Characteristic 报警通知控制点";
	if(m == 0x2A45) return "UUID Characteristic 未读警报状态";
	if(m == 0x2A46) return "UUID Characteristic 新警报";
	if(m == 0x2A47) return "UUID Characteristic 支持的新警报类别";
	if(m == 0x2A48) return "UUID Characteristic 支持的未读警报类别";
	if(m == 0x2A49) return "UUID Characteristic 血压功能";
	if(m == 0x2A4A) return "UUID Characteristic HID信息";
	if(m == 0x2A4B) return "UUID Characteristic 报告地图";
	if(m == 0x2A4C) return "UUID Characteristic HID控制点";
	if(m == 0x2A4D) return "UUID Characteristic 报告";
	if(m == 0x2A4E) return "UUID Characteristic 协议模式";
	if(m == 0x2A4F) return "UUID Characteristic 扫描间隔窗口";
	if(m == 0x2A50) return "UUID Characteristic 即插即用ID";
	if(m == 0x2A51) return "UUID Characteristic 葡萄糖功能";
	if(m == 0x2A52) return "UUID Characteristic 记录访问控制点";
	if(m == 0x2A53) return "UUID Characteristic RSC测量";
	if(m == 0x2A54) return "UUID Characteristic RSC功能";
	if(m == 0x2A55) return "UUID Characteristic SC控制点";
	if(m == 0x2A56) return "UUID Characteristic 数字";
	if(m == 0x2A57) return "UUID Characteristic 数字输出";
	if(m == 0x2A58) return "UUID Characteristic 模拟";
	if(m == 0x2A59) return "UUID Characteristic 模拟输出";
	if(m == 0x2A5A) return "UUID Characteristic 总数";
	if(m == 0x2A5B) return "UUID Characteristic CSC测量";
	if(m == 0x2A5C) return "UUID Characteristic CSC功能";
	if(m == 0x2A5D) return "UUID Characteristic 传感器位置";
	if(m == 0x2A5E) return "UUID Characteristic PLX抽查检查";
	if(m == 0x2A5F) return "UUID Characteristic PLX连续测量特性";
	if(m == 0x2A60) return "UUID Characteristic PLX功能";
	if(m == 0x2A62) return "UUID Characteristic 脉搏血氧饱和度控制点";
	if(m == 0x2A63) return "UUID Characteristic 骑行能量测量";
	if(m == 0x2A64) return "UUID Characteristic 骑行能量矢量";
	if(m == 0x2A65) return "UUID Characteristic 骑行能量功能";
	if(m == 0x2A66) return "UUID Characteristic 骑行能量控制点";
	if(m == 0x2A67) return "UUID Characteristic 位置和速度特征";
	if(m == 0x2A68) return "UUID Characteristic 导航";
	if(m == 0x2A69) return "UUID Characteristic 位置质量";
	if(m == 0x2A6A) return "UUID Characteristic LN功能";
	if(m == 0x2A6B) return "UUID Characteristic LN控制点";
	if(m == 0x2A6C) return "UUID Characteristic 海拔";
	if(m == 0x2A6D) return "UUID Characteristic 压力";
	if(m == 0x2A6E) return "UUID Characteristic 温度";
	if(m == 0x2A6F) return "UUID Characteristic 湿度";
	if(m == 0x2A70) return "UUID Characteristic 真风速";
	if(m == 0x2A71) return "UUID Characteristic 真风向";
	if(m == 0x2A72) return "UUID Characteristic 视风速";
	if(m == 0x2A73) return "UUID Characteristic 视风向";
	if(m == 0x2A74) return "UUID Characteristic 阵风系数";
	if(m == 0x2A75) return "UUID Characteristic 花粉浓度";
	if(m == 0x2A76) return "UUID Characteristic 紫外线指数";
	if(m == 0x2A77) return "UUID Characteristic 辐照度";
	if(m == 0x2A78) return "UUID Characteristic 雨量";
	if(m == 0x2A79) return "UUID Characteristic 风寒（系数吧）";
	if(m == 0x2A7A) return "UUID Characteristic 热度指数";
	if(m == 0x2A7B) return "UUID Characteristic 露点温度";
	if(m == 0x2A7D) return "UUID Characteristic 描述符值已更改";
	if(m == 0x2A7E) return "UUID Characteristic 有氧心律下限";
	if(m == 0x2A7F) return "UUID Characteristic 有氧运动阈值";
	if(m == 0x2A80) return "UUID Characteristic 年龄";
	if(m == 0x2A81) return "UUID Characteristic 无氧心率下限";
	if(m == 0x2A82) return "UUID Characteristic 无氧心率上限";
	if(m == 0x2A83) return "UUID Characteristic 无氧运动阈值";
	if(m == 0x2A84) return "UUID Characteristic 有氧心率上限";
	if(m == 0x2A85) return "UUID Characteristic 出生日期";
	if(m == 0x2A86) return "UUID Characteristic 阈值评估日期";
	if(m == 0x2A87) return "UUID Characteristic 电子邮件地址";
	if(m == 0x2A88) return "UUID Characteristic 脂肪燃烧心率下限";
	if(m == 0x2A89) return "UUID Characteristic 脂肪燃烧心率上限";
	if(m == 0x2A8A) return "UUID Characteristic 名字";
	if(m == 0x2A8B) return "UUID Characteristic 五区心率限制";
	if(m == 0x2A8C) return "UUID Characteristic 性别";
	if(m == 0x2A8D) return "UUID Characteristic 最大心率";
	if(m == 0x2A8E) return "UUID Characteristic 高度";
	if(m == 0x2A8F) return "UUID Characteristic 臀围";
	if(m == 0x2A90) return "UUID Characteristic 姓";
	if(m == 0x2A91) return "UUID Characteristic 推荐最大心率";
	if(m == 0x2A92) return "UUID Characteristic 静息心率";
	if(m == 0x2A93) return "UUID Characteristic 有氧阈值和无氧阈值的运动类型";
	if(m == 0x2A94) return "UUID Characteristic 三区心率限制";
	if(m == 0x2A95) return "UUID Characteristic 两区心率限制";
	if(m == 0x2A96) return "UUID Characteristic 最大摄氧量";
	if(m == 0x2A97) return "UUID Characteristic 腰围";
	if(m == 0x2A98) return "UUID Characteristic 重量";
	if(m == 0x2A99) return "UUID Characteristic 数据库更改增量";
	if(m == 0x2A9A) return "UUID Characteristic 用户索引";
	if(m == 0x2A9B) return "UUID Characteristic 身体组成特征";
	if(m == 0x2A9C) return "UUID Characteristic 身体组成测量";
	if(m == 0x2A9D) return "UUID Characteristic 体重测量";
	if(m == 0x2A9E) return "UUID Characteristic 体重秤功能";
	if(m == 0x2A9F) return "UUID Characteristic 用户控制点";
	if(m == 0x2AA0) return "UUID Characteristic 磁通密度–2D";
	if(m == 0x2AA1) return "UUID Characteristic 磁通密度–3D";
	if(m == 0x2AA2) return "UUID Characteristic 语言";
	if(m == 0x2AA3) return "UUID Characteristic 气压趋势";
	if(m == 0x2AA4) return "UUID Characteristic 绑定管理控制点";
	if(m == 0x2AA5) return "UUID Characteristic 绑定管理功能";
	if(m == 0x2AA6) return "UUID Characteristic 中央地址解析";
	if(m == 0x2AA7) return "UUID Characteristic CGM测量";
	if(m == 0x2AA8) return "UUID Characteristic CGM功能";
	if(m == 0x2AA9) return "UUID Characteristic CGM状态";
	if(m == 0x2AAA) return "UUID Characteristic CGM会话开始时间";
	if(m == 0x2AAB) return "UUID Characteristic CGM会话运行时间";
	if(m == 0x2AAC) return "UUID Characteristic CGM特定操作控制点";
	if(m == 0x2AAD) return "UUID Characteristic 室内定位配置";
	if(m == 0x2AAE) return "UUID Characteristic 纬度";
	if(m == 0x2AAF) return "UUID Characteristic 经度";
	if(m == 0x2AB0) return "UUID Characteristic 当地北部坐标";
	if(m == 0x2AB1) return "UUID Characteristic 当地东部坐标";
	if(m == 0x2AB2) return "UUID Characteristic 楼层号";
	if(m == 0x2AB3) return "UUID Characteristic 海拔";
	if(m == 0x2AB4) return "UUID Characteristic 不确定";
	if(m == 0x2AB5) return "UUID Characteristic 地点名称";
	if(m == 0x2AB6) return "UUID Characteristic URI链接";
	if(m == 0x2AB7) return "UUID Characteristic HTTP头";
	if(m == 0x2AB8) return "UUID Characteristic HTTP状态码";
	if(m == 0x2AB9) return "UUID Characteristic HTTP实体主体";
	if(m == 0x2ABA) return "UUID Characteristic HTTP控制点";
	if(m == 0x2ABB) return "UUID Characteristic HTTPS安全性";
	if(m == 0x2ABC) return "UUID Characteristic TDS控制点";
	if(m == 0x2ABD) return "UUID Characteristic OTS功能";
	if(m == 0x2ABE) return "UUID Characteristic 对象名称";
	if(m == 0x2ABF) return "UUID Characteristic 对象类型";
	if(m == 0x2AC0) return "UUID Characteristic 对象大小";
	if(m == 0x2AC1) return "UUID Characteristic 对象首先创建";
	if(m == 0x2AC2) return "UUID Characteristic 上次修改的对象";
	if(m == 0x2AC3) return "UUID Characteristic 对象ID";
	if(m == 0x2AC4) return "UUID Characteristic 对象属性";
	if(m == 0x2AC5) return "UUID Characteristic 对象动作控制点";
	if(m == 0x2AC6) return "UUID Characteristic 对象列表控制点";
	if(m == 0x2AC7) return "UUID Characteristic 对象列表过滤器";
	if(m == 0x2AC8) return "UUID Characteristic 对象已更改";
	if(m == 0x2AC9) return "UUID Characteristic 仅可解析的私有地址";
	if(m == 0x2ACC) return "UUID Characteristic 健身设备功能";
	if(m == 0x2ACD) return "UUID Characteristic 跑步机数据";
	if(m == 0x2ACE) return "UUID Characteristic 交叉训练员数据";
	if(m == 0x2ACF) return "UUID Characteristic 攀登者步数";
	if(m == 0x2AD0) return "UUID Characteristic 攀登楼梯数";
	if(m == 0x2AD1) return "UUID Characteristic 桨手数据";
	if(m == 0x2AD2) return "UUID Characteristic 室内自行车数据";
	if(m == 0x2AD3) return "UUID Characteristic 训练状况";
	if(m == 0x2AD4) return "UUID Characteristic 支持的速度范围";
	if(m == 0x2AD5) return "UUID Characteristic 支持的倾斜范围";
	if(m == 0x2AD6) return "UUID Characteristic 支持的电阻水平范围";
	if(m == 0x2AD7) return "UUID Characteristic 支持的心率范围";
	if(m == 0x2AD8) return "UUID Characteristic 支持的功率范围";
	if(m == 0x2AD9) return "UUID Characteristic 健身设备控制点";
	if(m == 0x2ADA) return "UUID Characteristic 健身设备状态";
	if(m == 0x2AED) return "UUID Characteristic UTC时间";
	if(m == 0x2B1D) return "UUID Characteristic RC功能";
	if(m == 0x2B1E) return "UUID Characteristic RC设置";
	if(m == 0x2B1F) return "UUID Characteristic 重新连接配置控制点";
	if(m == 0x2B20) return "UUID Characteristic IDD状态已更改";
	if(m == 0x2B21) return "UUID Characteristic IDD状态";
	if(m == 0x2B22) return "UUID Characteristic IDD通告状态";
	if(m == 0x2B23) return "UUID Characteristic IDD功能";
	if(m == 0x2B24) return "UUID Characteristic IDD状态读取器控制点";
	if(m == 0x2B25) return "UUID Characteristic IDD命令控制点";
	if(m == 0x2B26) return "UUID Characteristic IDD命令数据";
	if(m == 0x2B27) return "UUID Characteristic IDD记录访问控制点";
	if(m == 0x2B28) return "UUID Characteristic IDD历史数据";
	// BSS（Battery Service）：用于电池相关信息的服务，例如电量水平。
	if(m == 0x2B2B) return "UUID Characteristic BSS控制点";
	if(m == 0x2B2C) return "UUID Characteristic BSS回应";
	// EMCS（Environmental Monitoring Service）：用于环境监测的服务，例如温度、湿度等数据的传输。
	if(m == 0x2B2D) return "UUID Characteristic 突发事件ID";
	if(m == 0x2B2E) return "UUID Characteristic 突发事件内容";
	// UUID
	if(m == 0x2B37) return "UUID Characteristic 注册用户特征";

	// Descriptors
	if(m == 0x2900) return "UUID Descriptors 特性扩展属性";
	if(m == 0x2901) return "UUID Descriptors 特征用户描述";
	if(m == 0x2902) return "UUID Descriptors 客户端特征配置";
	if(m == 0x2903) return "UUID Descriptors 服务器特征配置";
	if(m == 0x2904) return "UUID Descriptors 特征描述格式";
	if(m == 0x2905) return "UUID Descriptors 特征汇总格式";
	if(m == 0x2906) return "UUID Descriptors 有效范围";
	if(m == 0x2907) return "UUID Descriptors 外部报告参考";
	if(m == 0x2908) return "UUID Descriptors 报告参考";
	if(m == 0x2909) return "UUID Descriptors 数字个数";
	if(m == 0x290A) return "UUID Descriptors 数值触发设定";
	if(m == 0x290B) return "UUID Descriptors 环境感应配置";
	if(m == 0x290C) return "UUID Descriptors 环境感测";
	if(m == 0x290D) return "UUID Descriptors 环境感应触发设定";
	if(m == 0x290E) return "UUID Descriptors 时间触发设定";

	static char ret[20];
	sprintf(ret, "UUID 未知 %d", m);
	return ret;
}

void BLE_GATT_MSG_DESC(gattMsgEvent_t *m)
{
	gattMsg_t msg = m->msg;
	if(m->method == ATT_ERROR_RSP) {
		PRINT("GATT {method:%s, {reqOpcode:%s, errCode:0x%02x}}", BLE_Opcode2str(m->method), BLE_Opcode2str(msg.errorRsp.reqOpcode), msg.errorRsp.errCode);
	} else if(m->method == ATT_EXCHANGE_MTU_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_EXCHANGE_MTU_RSP) {
		PRINT("GATT {method:%s, {clientRxMTU:%d}", BLE_Opcode2str(m->method), msg.exchangeMTUReq.clientRxMTU);
	} else if(m->method == ATT_FIND_INFO_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_FIND_INFO_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_FIND_BY_TYPE_VALUE_REQ) {
		// PRINT("GATT: method ATT_FIND_BY_TYPE_VALUE_REQ, {numInfo %d, pHandlesInfo 0x%02x}", msg.findByTypeValueReq.numInfo, *(msg.findByTypeValueReq.pHandlesInfo));
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_FIND_BY_TYPE_VALUE_RSP) {
		PRINT("GATT {method:%s, {numInfo:%d, pHandlesInfo:0x%02x}", BLE_Opcode2str(m->method), msg.findByTypeValueRsp.numInfo, msg.findByTypeValueRsp.pHandlesInfo);
	} else if(m->method == ATT_READ_BY_TYPE_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_BY_TYPE_RSP) {
		PRINT("GATT {method:%s, {numPairs %d, len %d, pDataList 0x", BLE_Opcode2str(m->method), msg.readByTypeRsp.numPairs, msg.readByTypeRsp.len);
		Print_Memory(msg.readByTypeRsp.pDataList, msg.readByTypeRsp.len);
		PRINT("}");
	} else if(m->method == ATT_READ_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_BLOB_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_BLOB_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_MULTI_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_MULTI_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_BY_GRP_TYPE_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_READ_BY_GRP_TYPE_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_WRITE_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_WRITE_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_PREPARE_WRITE_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_PREPARE_WRITE_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_EXECUTE_WRITE_REQ) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_EXECUTE_WRITE_RSP) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_HANDLE_VALUE_NOTI) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_HANDLE_VALUE_IND) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else if(m->method == ATT_HANDLE_VALUE_CFM) {
		PRINT("GATT {method:%s}", BLE_Opcode2str(m->method));
	} else {
		PRINT("GATT method unknown %d\r\n", m->method);
	}
}

void BLE_UUID_DESC(uint8_t *pDataList, uint16_t numGrps)
{
	for (uint16_t i = 0; i < numGrps; i++)
	{
		// 解析属性句柄
		uint16_t handle = pDataList[i * 6] | (pDataList[i * 6 + 1] << 8);
		// 解析结束组句柄
		uint16_t endGroupHandle = pDataList[i * 6 + 2] | (pDataList[i * 6 + 3] << 8);
		// 解析 UUID
		uint16_t uuid = pDataList[i * 6 + 4] | (pDataList[i * 6 + 5] << 8);
		// 输出解析结果
		PRINT("Service %d:\r\n", i + 1);
		PRINT("  Handle: 0x%04X\r\n", handle);
		PRINT("  End Group Handle: 0x%04X\r\n", endGroupHandle);
		PRINT("  UUID: 0x%04X %s\r\n", uuid, BLE_UUID2str(uuid));
	}
}
