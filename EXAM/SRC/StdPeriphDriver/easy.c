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
 * ���е�UUID��Ψһ
 * 1. GATT Service������ 16 λ
 * 2. GATT Characteristic�������� ����Ļ���Ԫ�أ�����������ֵ�����ֵ��ص����ԡ����������ǿɶ��ġ���д�Ļ�֪ͨ�ġ� 16 λ�� 128 λ��
 * 3. GATT Descriptors���������� �ṩ�����ĸ�����Ϣ�����ԣ�ͨ�����ڶ�����δ����������ݡ����磬��������������˵��һ�����������û����ԡ�  16 λ�� 128 λ��
 *
 * ����Service������һ������������Characteristic����
 * ÿ������������һ��������������Descriptor�����ṩ������Ϣ��
 * 
 * ����
 * �������ʷ���UUID: 0x180D��
 *   ���������ʲ���������UUID: 0x2A37��
 *     ���������û���������UUID: 0x2901������˵�����ʲ��������ݡ�
 * 
 * http://www.atmcu.com/2280.html
*/
const char* BLE_UUID2str(uint16_t m)
{
	// GATT ����
	if(m == 0x2800) return "UUID Declarations ��Ҫ����";
	if(m == 0x2801) return "UUID Declarations ��Ҫ����";
	if(m == 0x2802) return "UUID Declarations ����";
	if(m == 0x2803) return "UUID Declarations ��������";

	// Service
	// GSS��Generic Access Profile Service���������豸��ͨ�÷������á�
	if(m == 0x1800) return "UUID Service ͨ�÷���";
	if(m == 0x1801) return "UUID Service ͨ������";
	if(m == 0x1802) return "UUID Service ��ʱ����";
	if(m == 0x1803) return "UUID Service ���Ӷ�ʧ";
	if(m == 0x1804) return "UUID Service ���͹���";
	if(m == 0x1805) return "UUID Service ��ǰʱ��";
	if(m == 0x1806) return "UUID Service ����ʱ�����";
	if(m == 0x1807) return "UUID Service ����ʱ����";
	if(m == 0x1808) return "UUID Service ������";
	if(m == 0x1809) return "UUID Service �¶ȼ�";
	if(m == 0x180A) return "UUID Service �豸��Ϣ";
	if(m == 0x180D) return "UUID Service ����";
	if(m == 0x180E) return "UUID Service �ֻ�����״̬";
	if(m == 0x180F) return "UUID Service �������";
	if(m == 0x1810) return "UUID Service Ѫѹ";
	if(m == 0x1811) return "UUID Service ����֪ͨ";
	if(m == 0x1812) return "UUID Service HID�豸";
	if(m == 0x1813) return "UUID Service ɨ�����";
	if(m == 0x1814) return "UUID Service �ܲ��ٶȡ�����";
	if(m == 0x1815) return "UUID Service �Զ����������";
	if(m == 0x1816) return "UUID Service ѭ���ٶȡ�����";
	if(m == 0x1818) return "UUID Service ��������";
	if(m == 0x1819) return "UUID Service ��λ������";
	if(m == 0x181A) return "UUID Service ��������";
	if(m == 0x181B) return "UUID Service �������";
	if(m == 0x181C) return "UUID Service �û�����";
	if(m == 0x181D) return "UUID Service ���س�";
	if(m == 0x181E) return "UUID Service �豸�󶨹���";
	if(m == 0x181F) return "UUID Service ��̬Ѫ�Ǽ��";
	if(m == 0x1820) return "UUID Service ������Э��֧��";
	if(m == 0x1821) return "UUID Service ���ڶ�λ";
	if(m == 0x1822) return "UUID Service ����Ѫ����";
	if(m == 0x1823) return "UUID Service HTTP����";
	if(m == 0x1824) return "UUID Service ���䷢��";
	if(m == 0x1825) return "UUID Service ������";
	if(m == 0x1826) return "UUID Service �����豸";
	if(m == 0x1827) return "UUID Service �ڵ�����";
	if(m == 0x1828) return "UUID Service �ڵ����";
	if(m == 0x1829) return "UUID Service ��������";
	if(m == 0x183A) return "UUID Service �ȵ��ظ�ҩ";
	// BSS��Battery Service�������ڵ�������Ϣ�ķ����������ˮƽ��
	if(m == 0x183B) return "UUID Service ��Ԫ������";
	// EMCS��Environmental Monitoring Service�������ڻ������ķ��������¶ȡ�ʪ�ȵ����ݵĴ��䡣
	if(m == 0x183C) return "UUID Service Ӧ������";

	// Characteristic
	// GSS��Generic Access Profile Service���������豸��ͨ�÷������á�
	if(m == 0x2A00) return "UUID Characteristic �豸����";
	if(m == 0x2A01) return "UUID Characteristic ���";
	if(m == 0x2A02) return "UUID Characteristic �ܱ���˽��־";
	if(m == 0x2A03) return "UUID Characteristic �������ӵ�ַ";
	if(m == 0x2A04) return "UUID Characteristic ��Χ�豸��ѡ���Ӳ���";
	if(m == 0x2A05) return "UUID Characteristic �����Ѹ���";
	if(m == 0x2A06) return "UUID Characteristic �����ȼ�";
	if(m == 0x2A07) return "UUID Characteristic ���书�ʵȼ�";
	if(m == 0x2A08) return "UUID Characteristic ����ʱ��";
	if(m == 0x2A09) return "UUID Characteristic ���ڼ�";
	if(m == 0x2A0A) return "UUID Characteristic ����ʱ����";
	if(m == 0x2A0B) return "UUID Characteristic ����ʱ��100";
	if(m == 0x2A0C) return "UUID Characteristic ����ʱ��256";
	if(m == 0x2A0D) return "UUID Characteristic ����ʱƫ��";
	if(m == 0x2A0E) return "UUID Characteristic ʱ��";
	if(m == 0x2A0F) return "UUID Characteristic ����ʱ����Ϣ";
	if(m == 0x2A10) return "UUID Characteristic ��Ҫʱ��";
	if(m == 0x2A11) return "UUID Characteristic ����ʱ";
	if(m == 0x2A12) return "UUID Characteristic ʱ�侫��";
	if(m == 0x2A13) return "UUID Characteristic ʱ����Դ";
	if(m == 0x2A14) return "UUID Characteristic �ο�ʱ����Ϣ";
	if(m == 0x2A15) return "UUID Characteristic ʱ��㲥";
	if(m == 0x2A16) return "UUID Characteristic ʱ����¿��Ƶ�";
	if(m == 0x2A17) return "UUID Characteristic ʱ�����״̬";
	if(m == 0x2A18) return "UUID Characteristic Ѫ�ǲ���";
	if(m == 0x2A19) return "UUID Characteristic ��ص���";
	if(m == 0x2A1A) return "UUID Characteristic ��ص���״̬";
	if(m == 0x2A1B) return "UUID Characteristic ��ص����ȼ�";
	if(m == 0x2A1C) return "UUID Characteristic �¶Ȳ���";
	if(m == 0x2A1D) return "UUID Characteristic �¶�����";
	if(m == 0x2A1E) return "UUID Characteristic �м���¶�";
	if(m == 0x2A1F) return "UUID Characteristic �¶�����";
	if(m == 0x2A20) return "UUID Characteristic �¶Ȼ��϶�";
	if(m == 0x2A21) return "UUID Characteristic �������";
	if(m == 0x2A22) return "UUID Characteristic �����������뱨��";
	if(m == 0x2A23) return "UUID Characteristic ϵͳ���";
	if(m == 0x2A24) return "UUID Characteristic �ͺ��ַ�";
	if(m == 0x2A25) return "UUID Characteristic ���к��ַ�";
	if(m == 0x2A26) return "UUID Characteristic �̼��޶��ַ�";
	if(m == 0x2A27) return "UUID Characteristic Ӳ���޶��ַ�";
	if(m == 0x2A28) return "UUID Characteristic ����޶����ַ�";
	if(m == 0x2A29) return "UUID Characteristic �����������ַ�";
	if(m == 0x2A2A) return "UUID Characteristic 11073-20601������֤�����б�";
	if(m == 0x2A2B) return "UUID Characteristic ��ǰʱ��";
	if(m == 0x2A2C) return "UUID Characteristic ��ƫ��";
	if(m == 0x2A2F) return "UUID Characteristic λ��2D";
	if(m == 0x2A30) return "UUID Characteristic λ��3D";
	if(m == 0x2A31) return "UUID Characteristic ɨ��ˢ��";
	if(m == 0x2A32) return "UUID Characteristic ���������������";
	if(m == 0x2A33) return "UUID Characteristic ����������뱨��";
	if(m == 0x2A34) return "UUID Characteristic �����ǲ�������";
	if(m == 0x2A35) return "UUID Characteristic Ѫѹ����";
	if(m == 0x2A36) return "UUID Characteristic �м������ѹ��";
	if(m == 0x2A37) return "UUID Characteristic ���ʲ���";
	if(m == 0x2A38) return "UUID Characteristic �����Ӧ��λ��";
	if(m == 0x2A39) return "UUID Characteristic ���ʿ��Ƶ�";
	if(m == 0x2A3A) return "UUID Characteristic ���ƶ���";
	if(m == 0x2A3B) return "UUID Characteristic �������";
	if(m == 0x2A3C) return "UUID Characteristic ��ѧ�¶ȣ����϶ȣ�";
	if(m == 0x2A3D) return "UUID Characteristic �ַ���";
	if(m == 0x2A3E) return "UUID Characteristic ���������";
	if(m == 0x2A3F) return "UUID Characteristic ����״̬";
	if(m == 0x2A40) return "UUID Characteristic �������Ƶ�";
	if(m == 0x2A41) return "UUID Characteristic ��������";
	if(m == 0x2A42) return "UUID Characteristic �������IDλ����";
	if(m == 0x2A43) return "UUID Characteristic �������ID";
	if(m == 0x2A44) return "UUID Characteristic ����֪ͨ���Ƶ�";
	if(m == 0x2A45) return "UUID Characteristic δ������״̬";
	if(m == 0x2A46) return "UUID Characteristic �¾���";
	if(m == 0x2A47) return "UUID Characteristic ֧�ֵ��¾������";
	if(m == 0x2A48) return "UUID Characteristic ֧�ֵ�δ���������";
	if(m == 0x2A49) return "UUID Characteristic Ѫѹ����";
	if(m == 0x2A4A) return "UUID Characteristic HID��Ϣ";
	if(m == 0x2A4B) return "UUID Characteristic �����ͼ";
	if(m == 0x2A4C) return "UUID Characteristic HID���Ƶ�";
	if(m == 0x2A4D) return "UUID Characteristic ����";
	if(m == 0x2A4E) return "UUID Characteristic Э��ģʽ";
	if(m == 0x2A4F) return "UUID Characteristic ɨ��������";
	if(m == 0x2A50) return "UUID Characteristic ���弴��ID";
	if(m == 0x2A51) return "UUID Characteristic �����ǹ���";
	if(m == 0x2A52) return "UUID Characteristic ��¼���ʿ��Ƶ�";
	if(m == 0x2A53) return "UUID Characteristic RSC����";
	if(m == 0x2A54) return "UUID Characteristic RSC����";
	if(m == 0x2A55) return "UUID Characteristic SC���Ƶ�";
	if(m == 0x2A56) return "UUID Characteristic ����";
	if(m == 0x2A57) return "UUID Characteristic �������";
	if(m == 0x2A58) return "UUID Characteristic ģ��";
	if(m == 0x2A59) return "UUID Characteristic ģ�����";
	if(m == 0x2A5A) return "UUID Characteristic ����";
	if(m == 0x2A5B) return "UUID Characteristic CSC����";
	if(m == 0x2A5C) return "UUID Characteristic CSC����";
	if(m == 0x2A5D) return "UUID Characteristic ������λ��";
	if(m == 0x2A5E) return "UUID Characteristic PLX�����";
	if(m == 0x2A5F) return "UUID Characteristic PLX������������";
	if(m == 0x2A60) return "UUID Characteristic PLX����";
	if(m == 0x2A62) return "UUID Characteristic ����Ѫ�����Ͷȿ��Ƶ�";
	if(m == 0x2A63) return "UUID Characteristic ������������";
	if(m == 0x2A64) return "UUID Characteristic ��������ʸ��";
	if(m == 0x2A65) return "UUID Characteristic ������������";
	if(m == 0x2A66) return "UUID Characteristic �����������Ƶ�";
	if(m == 0x2A67) return "UUID Characteristic λ�ú��ٶ�����";
	if(m == 0x2A68) return "UUID Characteristic ����";
	if(m == 0x2A69) return "UUID Characteristic λ������";
	if(m == 0x2A6A) return "UUID Characteristic LN����";
	if(m == 0x2A6B) return "UUID Characteristic LN���Ƶ�";
	if(m == 0x2A6C) return "UUID Characteristic ����";
	if(m == 0x2A6D) return "UUID Characteristic ѹ��";
	if(m == 0x2A6E) return "UUID Characteristic �¶�";
	if(m == 0x2A6F) return "UUID Characteristic ʪ��";
	if(m == 0x2A70) return "UUID Characteristic �����";
	if(m == 0x2A71) return "UUID Characteristic �����";
	if(m == 0x2A72) return "UUID Characteristic �ӷ���";
	if(m == 0x2A73) return "UUID Characteristic �ӷ���";
	if(m == 0x2A74) return "UUID Characteristic ���ϵ��";
	if(m == 0x2A75) return "UUID Characteristic ����Ũ��";
	if(m == 0x2A76) return "UUID Characteristic ������ָ��";
	if(m == 0x2A77) return "UUID Characteristic ���ն�";
	if(m == 0x2A78) return "UUID Characteristic ����";
	if(m == 0x2A79) return "UUID Characteristic �纮��ϵ���ɣ�";
	if(m == 0x2A7A) return "UUID Characteristic �ȶ�ָ��";
	if(m == 0x2A7B) return "UUID Characteristic ¶���¶�";
	if(m == 0x2A7D) return "UUID Characteristic ������ֵ�Ѹ���";
	if(m == 0x2A7E) return "UUID Characteristic ������������";
	if(m == 0x2A7F) return "UUID Characteristic �����˶���ֵ";
	if(m == 0x2A80) return "UUID Characteristic ����";
	if(m == 0x2A81) return "UUID Characteristic ������������";
	if(m == 0x2A82) return "UUID Characteristic ������������";
	if(m == 0x2A83) return "UUID Characteristic �����˶���ֵ";
	if(m == 0x2A84) return "UUID Characteristic ������������";
	if(m == 0x2A85) return "UUID Characteristic ��������";
	if(m == 0x2A86) return "UUID Characteristic ��ֵ��������";
	if(m == 0x2A87) return "UUID Characteristic �����ʼ���ַ";
	if(m == 0x2A88) return "UUID Characteristic ֬��ȼ����������";
	if(m == 0x2A89) return "UUID Characteristic ֬��ȼ����������";
	if(m == 0x2A8A) return "UUID Characteristic ����";
	if(m == 0x2A8B) return "UUID Characteristic ������������";
	if(m == 0x2A8C) return "UUID Characteristic �Ա�";
	if(m == 0x2A8D) return "UUID Characteristic �������";
	if(m == 0x2A8E) return "UUID Characteristic �߶�";
	if(m == 0x2A8F) return "UUID Characteristic ��Χ";
	if(m == 0x2A90) return "UUID Characteristic ��";
	if(m == 0x2A91) return "UUID Characteristic �Ƽ��������";
	if(m == 0x2A92) return "UUID Characteristic ��Ϣ����";
	if(m == 0x2A93) return "UUID Characteristic ������ֵ��������ֵ���˶�����";
	if(m == 0x2A94) return "UUID Characteristic ������������";
	if(m == 0x2A95) return "UUID Characteristic ������������";
	if(m == 0x2A96) return "UUID Characteristic ���������";
	if(m == 0x2A97) return "UUID Characteristic ��Χ";
	if(m == 0x2A98) return "UUID Characteristic ����";
	if(m == 0x2A99) return "UUID Characteristic ���ݿ��������";
	if(m == 0x2A9A) return "UUID Characteristic �û�����";
	if(m == 0x2A9B) return "UUID Characteristic �����������";
	if(m == 0x2A9C) return "UUID Characteristic ������ɲ���";
	if(m == 0x2A9D) return "UUID Characteristic ���ز���";
	if(m == 0x2A9E) return "UUID Characteristic ���سӹ���";
	if(m == 0x2A9F) return "UUID Characteristic �û����Ƶ�";
	if(m == 0x2AA0) return "UUID Characteristic ��ͨ�ܶȨC2D";
	if(m == 0x2AA1) return "UUID Characteristic ��ͨ�ܶȨC3D";
	if(m == 0x2AA2) return "UUID Characteristic ����";
	if(m == 0x2AA3) return "UUID Characteristic ��ѹ����";
	if(m == 0x2AA4) return "UUID Characteristic �󶨹�����Ƶ�";
	if(m == 0x2AA5) return "UUID Characteristic �󶨹�����";
	if(m == 0x2AA6) return "UUID Characteristic �����ַ����";
	if(m == 0x2AA7) return "UUID Characteristic CGM����";
	if(m == 0x2AA8) return "UUID Characteristic CGM����";
	if(m == 0x2AA9) return "UUID Characteristic CGM״̬";
	if(m == 0x2AAA) return "UUID Characteristic CGM�Ự��ʼʱ��";
	if(m == 0x2AAB) return "UUID Characteristic CGM�Ự����ʱ��";
	if(m == 0x2AAC) return "UUID Characteristic CGM�ض��������Ƶ�";
	if(m == 0x2AAD) return "UUID Characteristic ���ڶ�λ����";
	if(m == 0x2AAE) return "UUID Characteristic γ��";
	if(m == 0x2AAF) return "UUID Characteristic ����";
	if(m == 0x2AB0) return "UUID Characteristic ���ر�������";
	if(m == 0x2AB1) return "UUID Characteristic ���ض�������";
	if(m == 0x2AB2) return "UUID Characteristic ¥���";
	if(m == 0x2AB3) return "UUID Characteristic ����";
	if(m == 0x2AB4) return "UUID Characteristic ��ȷ��";
	if(m == 0x2AB5) return "UUID Characteristic �ص�����";
	if(m == 0x2AB6) return "UUID Characteristic URI����";
	if(m == 0x2AB7) return "UUID Characteristic HTTPͷ";
	if(m == 0x2AB8) return "UUID Characteristic HTTP״̬��";
	if(m == 0x2AB9) return "UUID Characteristic HTTPʵ������";
	if(m == 0x2ABA) return "UUID Characteristic HTTP���Ƶ�";
	if(m == 0x2ABB) return "UUID Characteristic HTTPS��ȫ��";
	if(m == 0x2ABC) return "UUID Characteristic TDS���Ƶ�";
	if(m == 0x2ABD) return "UUID Characteristic OTS����";
	if(m == 0x2ABE) return "UUID Characteristic ��������";
	if(m == 0x2ABF) return "UUID Characteristic ��������";
	if(m == 0x2AC0) return "UUID Characteristic �����С";
	if(m == 0x2AC1) return "UUID Characteristic �������ȴ���";
	if(m == 0x2AC2) return "UUID Characteristic �ϴ��޸ĵĶ���";
	if(m == 0x2AC3) return "UUID Characteristic ����ID";
	if(m == 0x2AC4) return "UUID Characteristic ��������";
	if(m == 0x2AC5) return "UUID Characteristic ���������Ƶ�";
	if(m == 0x2AC6) return "UUID Characteristic �����б���Ƶ�";
	if(m == 0x2AC7) return "UUID Characteristic �����б������";
	if(m == 0x2AC8) return "UUID Characteristic �����Ѹ���";
	if(m == 0x2AC9) return "UUID Characteristic ���ɽ�����˽�е�ַ";
	if(m == 0x2ACC) return "UUID Characteristic �����豸����";
	if(m == 0x2ACD) return "UUID Characteristic �ܲ�������";
	if(m == 0x2ACE) return "UUID Characteristic ����ѵ��Ա����";
	if(m == 0x2ACF) return "UUID Characteristic �ʵ��߲���";
	if(m == 0x2AD0) return "UUID Characteristic �ʵ�¥����";
	if(m == 0x2AD1) return "UUID Characteristic ��������";
	if(m == 0x2AD2) return "UUID Characteristic �������г�����";
	if(m == 0x2AD3) return "UUID Characteristic ѵ��״��";
	if(m == 0x2AD4) return "UUID Characteristic ֧�ֵ��ٶȷ�Χ";
	if(m == 0x2AD5) return "UUID Characteristic ֧�ֵ���б��Χ";
	if(m == 0x2AD6) return "UUID Characteristic ֧�ֵĵ���ˮƽ��Χ";
	if(m == 0x2AD7) return "UUID Characteristic ֧�ֵ����ʷ�Χ";
	if(m == 0x2AD8) return "UUID Characteristic ֧�ֵĹ��ʷ�Χ";
	if(m == 0x2AD9) return "UUID Characteristic �����豸���Ƶ�";
	if(m == 0x2ADA) return "UUID Characteristic �����豸״̬";
	if(m == 0x2AED) return "UUID Characteristic UTCʱ��";
	if(m == 0x2B1D) return "UUID Characteristic RC����";
	if(m == 0x2B1E) return "UUID Characteristic RC����";
	if(m == 0x2B1F) return "UUID Characteristic �����������ÿ��Ƶ�";
	if(m == 0x2B20) return "UUID Characteristic IDD״̬�Ѹ���";
	if(m == 0x2B21) return "UUID Characteristic IDD״̬";
	if(m == 0x2B22) return "UUID Characteristic IDDͨ��״̬";
	if(m == 0x2B23) return "UUID Characteristic IDD����";
	if(m == 0x2B24) return "UUID Characteristic IDD״̬��ȡ�����Ƶ�";
	if(m == 0x2B25) return "UUID Characteristic IDD������Ƶ�";
	if(m == 0x2B26) return "UUID Characteristic IDD��������";
	if(m == 0x2B27) return "UUID Characteristic IDD��¼���ʿ��Ƶ�";
	if(m == 0x2B28) return "UUID Characteristic IDD��ʷ����";
	// BSS��Battery Service�������ڵ�������Ϣ�ķ����������ˮƽ��
	if(m == 0x2B2B) return "UUID Characteristic BSS���Ƶ�";
	if(m == 0x2B2C) return "UUID Characteristic BSS��Ӧ";
	// EMCS��Environmental Monitoring Service�������ڻ������ķ��������¶ȡ�ʪ�ȵ����ݵĴ��䡣
	if(m == 0x2B2D) return "UUID Characteristic ͻ���¼�ID";
	if(m == 0x2B2E) return "UUID Characteristic ͻ���¼�����";
	// UUID
	if(m == 0x2B37) return "UUID Characteristic ע���û�����";

	// Descriptors
	if(m == 0x2900) return "UUID Descriptors ������չ����";
	if(m == 0x2901) return "UUID Descriptors �����û�����";
	if(m == 0x2902) return "UUID Descriptors �ͻ�����������";
	if(m == 0x2903) return "UUID Descriptors ��������������";
	if(m == 0x2904) return "UUID Descriptors ����������ʽ";
	if(m == 0x2905) return "UUID Descriptors �������ܸ�ʽ";
	if(m == 0x2906) return "UUID Descriptors ��Ч��Χ";
	if(m == 0x2907) return "UUID Descriptors �ⲿ����ο�";
	if(m == 0x2908) return "UUID Descriptors ����ο�";
	if(m == 0x2909) return "UUID Descriptors ���ָ���";
	if(m == 0x290A) return "UUID Descriptors ��ֵ�����趨";
	if(m == 0x290B) return "UUID Descriptors ������Ӧ����";
	if(m == 0x290C) return "UUID Descriptors �����в�";
	if(m == 0x290D) return "UUID Descriptors ������Ӧ�����趨";
	if(m == 0x290E) return "UUID Descriptors ʱ�䴥���趨";

	static char ret[20];
	sprintf(ret, "UUID δ֪ %d", m);
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
		// �������Ծ��
		uint16_t handle = pDataList[i * 6] | (pDataList[i * 6 + 1] << 8);
		// ������������
		uint16_t endGroupHandle = pDataList[i * 6 + 2] | (pDataList[i * 6 + 3] << 8);
		// ���� UUID
		uint16_t uuid = pDataList[i * 6 + 4] | (pDataList[i * 6 + 5] << 8);
		// ����������
		PRINT("Service %d:\r\n", i + 1);
		PRINT("  Handle: 0x%04X\r\n", handle);
		PRINT("  End Group Handle: 0x%04X\r\n", endGroupHandle);
		PRINT("  UUID: 0x%04X %s\r\n", uuid, BLE_UUID2str(uuid));
	}
}
