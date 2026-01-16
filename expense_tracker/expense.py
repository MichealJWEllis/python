"""
Expense Module
--------------
Defines the Expense class for representing individual expense transactions.
"""

class Expense:
    """
    Represents a single expense transaction.
    
    Attributes:
        amount (float): The cost of the expense
        category (str): Category of the expense (e.g., 'Groceries', 'Transportation')
        description (str): Brief description of what the expense was for
        payment_method (str): How it was paid (stored lowercase: 'cash', 'credit', etc.)
        date (str): Date of expense in YYYY-MM-DD format
    """
    
    def __init__(self, amount, category, description, payment_method, date):
        """
        Initialize a new Expense.
        
        Args:
            amount (float): The cost of the expense
            category (str): Category of the expense
            description (str): What the expense was for
            payment_method (str): Payment method (automatically converted to lowercase)
            date (str): Date in YYYY-MM-DD format
        """
        self.amount = amount
        self.category = category
        self.description = description
        self.payment_method = payment_method.lower()  # Normalize to lowercase
        self.date = date
    
    def __str__(self):
        """
        Returns a formatted string representation of the expense.
        
        Returns:
            str: Human-readable expense summary
        """
        return f"${self.amount} - {self.category} - {self.description} - Paid with {self.payment_method} on date: {self.date}"
    
    def is_cash(self):
        """
        Check if the expense was paid with cash.
        
        Returns:
            bool: True if payment method is 'cash', False otherwise
        """
        return self.payment_method == "cash"


# Test code - only runs when file is executed directly
if __name__ == '__main__':
    expense1 = Expense(45.00, "Grocceries", "weekly Shopping", "Cash", "2024-01-10")
    expense2 = Expense(12.50, "Transportation", "Uber to work", "Credit", "2025-01-10")
    expense3 = Expense(50.00, "Subscriptions", "Claude", "Credit", "2025-01-10")

    print(expense1)
    print(expense2)
    print(expense3)