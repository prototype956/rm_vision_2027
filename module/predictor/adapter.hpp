#ifndef PREDICTOR_ADAPTER_HPP
#define PREDICTOR_ADAPTER_HPP

#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/predictor/armor.hpp"

namespace predictor {

/**
 * @brief 将 basic_armor::Armor_Data 转换为 predictor::Armor
 * @param basic_data 基础装甲板数据
 * @param receive_data 串口接收的数据（用于获取我方颜色）
 * @return 转换后的 predictor::Armor 对象
 */
inline Armor convertToPredictorArmor(const basic_armor::Armor_Data& basic_data,
                                     const uart::Receive_Data& receive_data) {
  // 1. 构造左右灯条 Lightbar
  // 这里我们暂时给左灯条ID为0，右灯条ID为1
  Lightbar left_bar(basic_data.left_light, 1);
  Lightbar right_bar(basic_data.right_light, 0);

  // 2. 使用左右灯条构造 Armor
  // 这个构造函数会自动计算装甲板的 center, ratio, angle 等几何参数
  Armor armor(left_bar, right_bar);

  // 3. 转换装甲板类型 (distinguish: 0 小装甲板, 1 大装甲板)
  if (basic_data.distinguish == 1) {
    armor.type = ArmorType::big;
  } else {
    armor.type = ArmorType::small;
  }

  // 4. 设置装甲板颜色 (反转逻辑：我方红->敌方蓝)
  // uart::Color: RED=1, BLUE=2
  // predictor::Color: red=0, blue=1
  if (receive_data.my_color == uart::RED)  // 我方是红色
  {
    armor.color = Color::blue;                     // 敌方是蓝色
  } else if (receive_data.my_color == uart::BLUE)  // 我方是蓝色
  {
    armor.color = Color::red;  // 敌方是红色
  } else {
    armor.color = Color::extinguish;  // 默认或未知
  }

  // 5. 设置数字编号
  // 由于暂时没有数字识别功能，统一设置为 not_armor 或默认值
  // 这会导致 Tracker 使用通用的 4 板模型参数
  armor.name = ArmorName::not_armor;

  // 6. 设置优先级
  // 由于没有数字，无法区分英雄/步兵优先级，暂时统一设为最高优先级
  armor.priority = ArmorPriority::first;

  // 7. 设置置信度
  armor.confidence = 1.0;

  return armor;
}

}  // namespace predictor

#endif  // PREDICTOR_ADAPTER_HPP
