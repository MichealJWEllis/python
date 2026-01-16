"""
Database Models
---------------
SQLAlchemy models for expense tracking database.
"""

from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

db = SQLAlchemy()

class Expense(db.Model):
    """
    Database model for expenses.
    
    Table: expenses
    """
    __tablename__ = 'expenses'
    
    id = db.Column(db.Integer, primary_key=True)
    amount = db.Column(db.Float, nullable=False)
    category = db.Column(db.String(100), nullable=False)
    description = db.Column(db.String(500), nullable=False)
    payment_method = db.Column(db.String(50), nullable=False)
    date = db.Column(db.String(10), nullable=False)  # YYYY-MM-DD format
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    
    def to_dict(self):
        """Convert expense to dictionary for JSON response."""
        return {
            'id': self.id,
            'amount': self.amount,
            'category': self.category,
            'description': self.description,
            'payment_method': self.payment_method,
            'date': self.date
        }
    
    def __repr__(self):
        return f"<Expense {self.id}: ${self.amount} - {self.category}>"