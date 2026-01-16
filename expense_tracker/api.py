"""
Expense Tracker REST API
------------------------
Flask-based REST API for managing expenses with SQLite database.

Endpoints:
    GET  /expenses         - List all expenses
    GET  /expenses/<id>    - Get single expense
    POST /expenses         - Create a new expense
    PUT  /expenses/<id>    - Update an expense
    DELETE /expenses/<id>  - Delete an expense
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from models import db, Expense
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
# Initialize Flask app
app = Flask(__name__)
CORS(app)

# Database configuration
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///expenses.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Initialize database
db.init_app(app)

# Get API key from environment
API_KEY = os.getenv('API_KEY')


def require_api_key():
    """Check if the request has a valid API key."""
    api_key = request.headers.get('X-API-Key')
    if api_key != API_KEY:
        return jsonify({"error": "Invalid or missing API key"}), 401
    return None


@app.route('/')
def hello():
    """Root endpoint - health check."""
    return "Expense Tracker API is running!"


@app.route('/expenses', methods=['GET'])
def get_expenses():
    """
    GET /expenses
    
    Returns all expenses, optionally filtered by category or payment_method.
    """
    auth_error = require_api_key()
    if auth_error:
        return auth_error
    
    # Start with all expenses
    query = Expense.query
    
    # Apply filters if provided
    category = request.args.get('category')
    payment = request.args.get('payment_method')
    
    if category:
        query = query.filter_by(category=category)
    if payment:
        query = query.filter_by(payment_method=payment)
    
    expenses = query.all()
    return jsonify([expense.to_dict() for expense in expenses]), 200


@app.route('/expenses/<int:id>', methods=['GET'])
def get_expense(id):
    """
    GET /expenses/<id>
    
    Returns a single expense by ID.
    """
    auth_error = require_api_key()
    if auth_error:
        return auth_error
    
    expense = Expense.query.get(id)
    if not expense:
        return jsonify({"error": "Expense not found"}), 404
    
    return jsonify(expense.to_dict()), 200


@app.route('/expenses', methods=['POST'])
def add_expense():
    """
    POST /expenses
    
    Create a new expense from JSON request body.
    """
    auth_error = require_api_key()
    if auth_error:
        return auth_error
    
    data = request.get_json()
    
    # Validate JSON exists
    if not data:
        return jsonify({"error": "Invalid or missing JSON"}), 400
    
    # Validate required fields
    required_fields = ["amount", "category", "description", "payment_method", "date"]
    for field in required_fields:
        if field not in data:
            return jsonify({"error": f"Missing field: {field}"}), 400
    
    # Create new expense
    expense = Expense(
        amount=data["amount"],
        category=data["category"],
        description=data["description"],
        payment_method=data["payment_method"].lower(),
        date=data["date"]
    )
    
    # Save to database
    db.session.add(expense)
    db.session.commit()
    
    return jsonify({
        "message": "Expense added successfully",
        "expense": expense.to_dict()
    }), 201


@app.route('/expenses/<int:id>', methods=['PUT'])
def update_expense(id):
    """
    PUT /expenses/<id>
    
    Update an expense by its ID.
    """
    auth_error = require_api_key()
    if auth_error:
        return auth_error
    
    expense = Expense.query.get(id)
    if not expense:
        return jsonify({"error": "Expense not found"}), 404
    
    data = request.get_json()
    
    # Update only provided fields
    if 'amount' in data:
        expense.amount = data['amount']
    if 'category' in data:
        expense.category = data['category']
    if 'description' in data:
        expense.description = data['description']
    if 'payment_method' in data:
        expense.payment_method = data['payment_method'].lower()
    if 'date' in data:
        expense.date = data['date']
    
    db.session.commit()
    
    return jsonify({
        "message": "Expense updated successfully",
        "expense": expense.to_dict()
    }), 200


@app.route('/expenses/<int:id>', methods=['DELETE'])
def delete_expense(id):
    """
    DELETE /expenses/<id>
    
    Delete an expense by its ID.
    """
    auth_error = require_api_key()
    if auth_error:
        return auth_error
    
    expense = Expense.query.get(id)
    if not expense:
        return jsonify({"error": "Expense not found"}), 404
    
    db.session.delete(expense)
    db.session.commit()
    
    return jsonify({"message": "Expense deleted successfully"}), 200


# Create database tables
with app.app_context():
    db.create_all()


# Run the Flask development server
if __name__ == '__main__':
    app.run(debug=True, port=5000)