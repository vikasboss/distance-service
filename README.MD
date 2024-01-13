# Order Sequence Optimization

This repository contains a Flask web application using Google OR-Tools to optimize the sequence of orders for a delivery vehicle based on their pickup and delivery locations.

## Contents

- **distance.py**: Python script containing the Flask web application.
- **requirements.txt**: List of Python dependencies, including ortools and Flask.
- **Sample Curl**: Example curl command to test the `/get_order_sequence` endpoint.
- **Sample Response**: Example response from the web application.

## Usage

1. Install the required packages by running:

   ```bash
   pip install -r requirements.txt
Run the Flask web application:

2. Run below command:
   ```bash
    python distance.py

3. Use the provided sample curl command to test the /get_order_sequence endpoint:

    ```bash   
    curl --location 'http://localhost:5002/get_order_sequence' \
    --header 'Content-Type: application/json' \
    --data '{
    "vehicle_current_lat": 12.39045491664713,
    "vehicle_current_long": 12.39045491664713,
    "orders": [
        {
            "index": 0,
            "orderUUID": "duehduheudheudh",
            "order_pickup_lat": 12.39045491664713,
            "order_pickup_long": 21.309895456125858,
            "order_delivery_lat": 22.39045491664713,
            "order_delivery_long": 31.309895456125858
        },
        {
            "index": 1,
            "orderUUID": "jfefnejnf",
            "order_pickup_lat": 37.92445947222828,
            "order_pickup_long": 34.95880227532126,
            "order_delivery_lat": 47.39045491664713,
            "order_delivery_long": 44.309895456125858
        },
        {
            "index": 2,
            "orderUUID": "iejifjeifjeifj",
            "order_pickup_lat": 34.46419459252078,
            "order_pickup_long": -40.231629577681076,
            "order_delivery_lat": 35.39045491664713,
            "order_delivery_long": -41.309895456125858
        }
    ]
    }'

4. Sample Response
The web application will respond with a JSON array representing the optimized order sequence.
    ```bash   
    [
        {
            "index": 0,
            "orderUUID": "duehduheudheudh",
            "order_delivery_lat": 22.39045491664713,
            "order_delivery_long": 31.309895456125858,
            "order_pickup_lat": 12.39045491664713,
            "order_pickup_long": 21.309895456125858
        },
        {
            "index": 2,
            "orderUUID": "iejifjeifjeifj",
            "order_delivery_lat": 35.39045491664713,
            "order_delivery_long": -41.30989545612586,
            "order_pickup_lat": 34.46419459252078,
            "order_pickup_long": -40.231629577681076
        },
        {
            "index": 1,
            "orderUUID": "jfefnejnf",
            "order_delivery_lat": 47.39045491664713,
            "order_delivery_long": 44.30989545612586,
            "order_pickup_lat": 37.92445947222828,
            "order_pickup_long": 34.95880227532126
        }
    ]


