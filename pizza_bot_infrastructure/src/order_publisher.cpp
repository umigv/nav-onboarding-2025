#include "order_publisher.h"

#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>

using json = nlohmann::json;
using namespace std::chrono_literals;

OrderPublisher::OrderPublisher()
    : Node("order_publisher_node"),
    _order_count(0),
    _first_order(true)
{
    declare_parameter("orders_path", "orders.json");
    declare_parameter("repeat_orders", true);
    declare_parameter("order_delay_seconds", 5);

    _order_publisher = create_publisher<pizza_bot_interfaces::msg::Order>("orders", 10);
    
    // Want first order to be published quickly, then introduce longer delay 
    // between subsequent orders
    _order_timer = create_wall_timer(1s, 
        std::bind(&OrderPublisher::publish_order, this));

    // Open and parse orders json file only once
    std::string orders_path = get_parameter("orders_path").as_string();
    RCLCPP_INFO(get_logger(),
        "Retrieving orders from %s",
        orders_path.c_str());
    std::ifstream orders_file(orders_path);
    json orders_json = json::parse(orders_file);
    _orders = orders_json["orders"];
}

void OrderPublisher::publish_order()
{
    if (_order_count >= _orders.size())
    {
        bool repeat_orders = get_parameter("repeat_orders").as_bool();
        if (repeat_orders)
        {
            _order_count = 0;
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                "---------- All orders published ----------");
            _order_timer->cancel();
            return;
        }
    }

    if (_first_order)
    {
        // After first order, introduce delay
        _first_order = false;
        int order_delay = get_parameter("order_delay_seconds").as_int();
        _order_timer = create_wall_timer(std::chrono::seconds(order_delay), 
            std::bind(&OrderPublisher::publish_order, this));
    }
    
    json order = _orders[_order_count];
    
    pizza_bot_interfaces::msg::Order order_message;
    order_message.order_id = _order_count;
    order_message.pizza_place = order["pizza_place"].template get<std::string>();
    order_message.pizza_type = order["pizza_type"].template get<std::string>();

    pizza_bot_interfaces::msg::Coord pizza_place_coord, customer_coord;
    pizza_place_coord.x = order["pizza_place_coord"][0];
    pizza_place_coord.y = order["pizza_place_coord"][1];
    customer_coord.x = order["customer_coord"][0];
    customer_coord.y = order["customer_coord"][1];

    order_message.pizza_place_coord = pizza_place_coord;
    order_message.customer_coord = customer_coord;

    RCLCPP_INFO(get_logger(),
        "---------- Publishing order ----------");
    _order_publisher->publish(order_message);
    ++_order_count;
}
